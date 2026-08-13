"""Nav2 на роботе: tf_setup + map_server + planner_server + lifecycle + local_costmap
+ simple_controller. Без RViz, без заезда, без стека CMU.

Намеренно НЕ здесь:
  Point-LIO   -> deploy/run_pointlio.sh
  matcher     -> deploy/run_matcher.sh (даёт TF map->camera_init)
  RViz        -> на ноутбуке через domain_bridge
  vel_ctrl    -> deploy/run_vel_ctrl.sh. Пока мост не запущен, робот не может
                 поехать — так проверяется вся цепочка при неподвижных лапах.
  ⛔ стек CMU (pathFollower / local_planner / far_planner): pathFollower публикует
     и /cmd_vel, и /api/sport/request напрямую -> второй водитель. Не запускать
     system_real_robot.launch вместе с Nav2.

⚠️ Карты берутся ПАРОЙ из одной сессии: 2D building_reloc2.yaml (здесь) + 3D
   reloc2_gravity.pcd (в go2_localization/config/localization.yaml) + сетка
   localizability wall_density_reloc2.locgrid. Подменить одну — матчер и
   планировщик окажутся в разных геометриях. Старые пары не мешать:
   (building_expF + expF_map), (building_old + final_map_lc), (building + loc_5_map).

Запуск на собаке: deploy/run_nav2.sh
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("go2_navigation")
    # живая карта = building_reloc2.yaml, пара к reloc2_gravity.pcd
    default_map = os.path.join(pkg, "maps", "building_reloc2.yaml")
    default_params = os.path.join(pkg, "config", "nav2_params.yaml")

    args = [
        DeclareLaunchArgument("map", default_value=default_map,
                              description="2D-карта для map_server (живая = building_reloc2.yaml)"),
        DeclareLaunchArgument("params", default_value=default_params,
                              description="конфигурация Nav2"),
    ]

    # недостающее звено дерева кадров: aft_mapped -> base_link
    tf_setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "tf_setup.launch.py")),
    )

    # 2D-карта как /map, защёлкнуто
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        parameters=[{"yaml_filename": LaunchConfiguration("map")}],
        output="screen",
    )

    # NavFn + global_costmap
    planner = Node(
        package="nav2_planner", executable="planner_server", name="planner_server",
        parameters=[LaunchConfiguration("params")],
        output="screen",
    )

    # ⛔ controller_server (RPP) не поднимается: его rotateToHeading релейный и на
    # этой собаке даёт устойчивый проскок, параметром не лечится. Вместо него
    # simple_controller.py с пропорциональным доворотом. Вернуть RPP = добавить узел
    # и "controller_server" в node_names, а simple_controller заменить на
    # goal_to_controller.py.

    # local_costmap отдельным узлом: она была под-узлом controller_server, и вместе
    # с ним пропала реакция на то, чего нет на статичной карте (люди, коробки).
    # Настройки лежат там же, в nav2_params.yaml.
    # ⚠️ Имя узла зашито внутри как /costmap/costmap, из launch не переименовать —
    #    отсюда и ключ в YAML "costmap: costmap:". Топики возвращаем к привычным
    #    /local_costmap/..., их ждут мост на ноутбук и RViz.
    local_costmap = Node(
        package="nav2_costmap_2d", executable="nav2_costmap_2d", name="costmap",
        parameters=[LaunchConfiguration("params")],
        remappings=[
            ("/costmap/costmap_raw", "/local_costmap/costmap_raw"),
            ("/costmap/costmap", "/local_costmap/costmap"),
            ("/costmap/costmap_updates", "/local_costmap/costmap_updates"),
        ],
        output="screen",
    )

    # порядок важен: карта -> планировщик
    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        parameters=[{
            "autostart": True,
            "node_names": ["map_server", "planner_server"],
        }],
        output="screen",
    )

    # У costmap свой дирижёр намеренно: Costmap2DROS на activate ждёт TF
    # map->base_link бесконечно, и в общем списке таймаут оборвал бы ВЕСЬ bringup —
    # остались бы без карты и планировщика. Так отказ локализован: планировать робот
    # сможет в любом случае, а костмап догонит, когда появится TF.
    lifecycle_costmap = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_costmap",
        parameters=[{
            "autostart": True,
            "node_names": ["costmap/costmap"],
        }],
        output="screen",
    )

    # /goal_pose (с ноутбука через domain_bridge) -> планировщик -> /cmd_vel
    goal_bridge = Node(
        package="go2_navigation", executable="simple_controller.py", name="simple_controller",
        output="screen",
    )

    return LaunchDescription(args + [tf_setup, map_server, planner,
                                     lifecycle, local_costmap, lifecycle_costmap,
                                     goal_bridge])
