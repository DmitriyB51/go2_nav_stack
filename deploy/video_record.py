#!/usr/bin/env python3
"""Видео с передней камеры Go2 -> файл на диске самой собаки (связь слабая, гнать
поток по воздуху бессмысленно; файл забираем потом через scp).

⭐ Ничего не декодируем и даже не разбираем сообщение: камера уже отдаёт H.264,
подписываемся с raw=True и режем полезную нагрузку по смещениям. Замерено на
роботе: штатный разбор полей -> 4.2 Гц (сыпет кадры), raw=True -> 28.4 Гц. Тут
важна дешевизна: перегруз записи однажды стоил расхождения Point-LIO на 2.5 км.

⚠️ Установленный unitree_go/msg/Go2FrontVideoData НЕВЕРНЫЙ. Реальная раскладка:
      0..3    заголовок CDR
      4..11   uint64 time_frame
      12..15  uint32 РАЗРЕШЕНИЕ (720/360)   <- .msg считает это длиной массива
      16..19  uint32 длина полезных данных
      20..    H.264 (Annex-B)
Штатный разбор выдаёт мусор (длины по 1.8 ГБ); этот код от .msg не зависит.

⚠️ 720p и 360p идут вперемешку в одном топике — склеенные в один файл не
откроются ничем. Пишем только выбранное --res.

Запуск на собаке:
    source ~/ros_env.sh
    python3 ~/video_record.py                  # 720p в ~/videos
    python3 ~/video_record.py --res 360        # втрое легче
    setsid nohup python3 ~/video_record.py > ~/videos/rec.log 2>&1 &   # переживёт ssh
    pkill -INT -f video_record.py              # именно -INT: нужен корректный конец файла

На ноутбуке:
    scp unitree@172.20.10.3:videos/<имя>.h264 .
    ffmpeg -f h264 -r <fps из сводки> -i <имя>.h264 -c copy <имя>.mp4
"""
import argparse
import os
import signal
import struct
import time

import rclpy
from rclpy.node import Node
from unitree_go.msg import Go2FrontVideoData

# смещения в сыром сообщении (раскладка — в шапке)
OFF_RES = 12        # uint32: 720 или 360
OFF_LEN = 16        # uint32: длина полезных данных
OFF_DATA = 20       # дальше — H.264

_stop = False


def _on_signal(signum, frame):
    """Только флаг — иначе не успеем корректно закрыть файл."""
    global _stop
    _stop = True


class VideoRecorder(Node):
    def __init__(self, path, want_res, min_free_mb):
        super().__init__("video_record")
        self.want_res = want_res
        self.min_free_mb = min_free_mb
        self.path = path
        self.fh = open(path, "wb")

        self.frames = 0          # записано кадров нужного разрешения
        self.other = 0           # другое разрешение
        self.bad = 0             # непохожая раскладка
        self.bytes = 0
        self.started = False     # дождались ли SPS
        self.t0 = time.time()
        self.t_log = self.t0
        self.disk_full = False

        # raw=True — ради этого всё и затевалось
        self.create_subscription(Go2FrontVideoData, "/frontvideostream",
                                 self.on_msg, 10, raw=True)
        self.create_timer(5.0, self.progress)
        self.get_logger().info("пишу %dp в %s" % (want_res, path))

    def free_mb(self):
        st = os.statvfs(os.path.dirname(self.path) or ".")
        return st.f_bavail * st.f_frsize / 1024 / 1024

    def on_msg(self, blob):
        if self.disk_full:
            return
        if len(blob) < OFF_DATA:
            self.bad += 1
            return
        res = struct.unpack_from("<I", blob, OFF_RES)[0]
        length = struct.unpack_from("<I", blob, OFF_LEN)[0]
        # длина обязана помещаться в сообщение, иначе раскладка не та
        if OFF_DATA + length > len(blob):
            self.bad += 1
            return
        if res != self.want_res:
            self.other += 1
            return

        data = blob[OFF_DATA:OFF_DATA + length]

        # ждём SPS (NAL-тип 7): без параметров кадра декодер не знает ни размера,
        # ни профиля. Робот шлёт SPS регулярно, ждать доли секунды
        if not self.started:
            if not self.has_sps(data):
                return
            self.started = True
            self.get_logger().info("поймал SPS — начинаю запись")

        self.fh.write(data)
        self.frames += 1
        self.bytes += len(data)

    @staticmethod
    def has_sps(data):
        """NAL-единица типа 7 сразу после старт-кода."""
        i = data.find(b"\x00\x00\x00\x01")
        while i >= 0 and i + 4 < len(data):
            if (data[i + 4] & 0x1f) == 7:
                return True
            i = data.find(b"\x00\x00\x00\x01", i + 4)
        return False

    def progress(self):
        now = time.time()
        d = now - self.t0
        free = self.free_mb()
        self.get_logger().info(
            "кадров %d (%.1f/с) | %.1f МБ | %.0f с | свободно %.0f МБ%s"
            % (self.frames, self.frames / d if d else 0, self.bytes / 1e6, d, free,
               "" if self.started else " | ЖДУ SPS"))
        # останавливаемся аккуратно, а не падаем на ENOSPC
        if free < self.min_free_mb:
            self.disk_full = True
            self.get_logger().error(
                "на диске осталось %.0f МБ — останавливаю запись" % free)
            global _stop
            _stop = True

    def finish(self):
        self.fh.flush()
        os.fsync(self.fh.fileno())
        self.fh.close()
        d = time.time() - self.t0
        fps = self.frames / d if d else 0
        summary = (
            "записано кадров : %d\n"
            "длительность    : %.1f с\n"
            "частота         : %.2f кадр/с   <- подставь это в ffmpeg -r\n"
            "размер          : %.1f МБ\n"
            "разрешение      : %dp\n"
            "пропущено (другое разрешение) : %d\n"
            "пропущено (битая раскладка)   : %d\n"
            "\nсобрать в mp4 на ноутбуке:\n"
            "  ffmpeg -f h264 -r %.2f -i %s -c copy %s.mp4\n"
            % (self.frames, d, fps, self.bytes / 1e6, self.want_res, self.other,
               self.bad, fps, os.path.basename(self.path),
               os.path.basename(self.path).rsplit(".", 1)[0]))
        # сводку кладём рядом: без fps ffmpeg соберёт mp4 с неправильной скоростью
        with open(self.path + ".txt", "w") as f:
            f.write(summary)
        print("\n===== ЗАПИСЬ ЗАВЕРШЕНА =====\n" + summary)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.expanduser("~/videos"),
                    help="куда писать (по умолчанию ~/videos)")
    ap.add_argument("--res", type=int, default=720, choices=(720, 360),
                    help="какое разрешение писать (они идут вперемешку)")
    ap.add_argument("--min-free-mb", type=float, default=500.0,
                    help="остановиться, когда на диске останется меньше стольких МБ")
    args = ap.parse_args()

    os.makedirs(args.dir, exist_ok=True)
    name = time.strftime("%Y-%m-%d_%H-%M-%S") + "_%dp.h264" % args.res
    path = os.path.join(args.dir, name)

    # свой Ctrl+C: rclpy гасит контекст первым и файл закроется как попало
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    node = VideoRecorder(path, args.res, args.min_free_mb)
    try:
        while rclpy.ok() and not _stop:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
