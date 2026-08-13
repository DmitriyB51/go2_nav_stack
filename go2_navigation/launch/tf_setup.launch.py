"""
ЗАПУСК:
  ros2 launch go2_navigation tf_setup.launch.py
  ros2 launch go2_navigation tf_setup.launch.py mount_yaw_deg:=-120.0 sensor_height:=0.42

ПРОВЕРКА:
  ros2 run tf2_tools view_frames          # дерево целиком, у каждого звена ОДИН издатель
  ros2 run tf2_ros tf2_echo camera_init base_link
"""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_static_tf(context, *args, **kwargs):
    """base_link в системе aft_mapped + статический TF.

    Крепление задаём понятными величинами (угол, высота, вынос), неочевидные x/y/z
    считаем здесь, чтобы в конфиге не было шести магических чисел.
    """
    # то же число, что heading_offset_deg в go2_localization
    mount_yaw_deg = float(LaunchConfiguration("mount_yaw_deg").perform(context))
    sensor_height = float(LaunchConfiguration("sensor_height").perform(context))
    sensor_forward = float(LaunchConfiguration("sensor_forward").perform(context))

    yaw = math.radians(mount_yaw_deg)

    # base_link из системы сенсора: на sensor_height ниже и на sensor_forward назад
    # вдоль "вперёд" робота (которое в кадре сенсора направлено под углом yaw)
    x = -sensor_forward * math.cos(yaw)
    y = -sensor_forward * math.sin(yaw)
    z = -sensor_height

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aft_mapped_to_base_link",
        output="screen",
        arguments=[
            "--x", str(x),
            "--y", str(y),
            "--z", str(z),
            "--yaw", str(yaw),
            "--pitch", "0",
            "--roll", "0",
            "--frame-id", "aft_mapped",        # кадр сенсора от Point-LIO
            "--child-frame-id", "base_link",   # корпус робота на полу
        ],
    )
    # aft_mapped -> body, тождественное: Point-LIO зовёт ОДИН кадр сенсора двумя
    # именами — в TF вещает aft_mapped (laserMapping.cpp:810), а облака штампует как
    # "body" (:744), которого в дереве нет ни у кого. Без этого звена obstacle_layer
    # молча выбрасывает каждое облако с "Message Filter dropping message: frame
    # 'body' ... earlier than all the data in the transform cache" (формулировка
    # обманчива: кэш пуст, потому что кадра не существует) -> костмап всегда пустой.
    body_alias_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aft_mapped_to_body",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--yaw", "0", "--pitch", "0", "--roll", "0",
            "--frame-id", "aft_mapped",   # имя кадра в TF
            "--child-frame-id", "body",   # имя кадра в облаках Point-LIO
        ],
    )

    return [static_tf, body_alias_tf]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mount_yaw_deg", default_value="-117.0",
            description="угол установки лидара относительно 'вперёд' робота, градусы "
                        "(то же, что heading_offset_deg; ТРЕБУЕТ калибровки на роботе)"),
        DeclareLaunchArgument(
            "sensor_height", default_value="0.40",
            description="высота лидара над полом, м"),
        DeclareLaunchArgument(
            "sensor_forward", default_value="0.30",
            description="вынос лидара вперёд от центра корпуса, м (sensorOffsetX у CMU)"),
        OpaqueFunction(function=make_static_tf),
    ])
