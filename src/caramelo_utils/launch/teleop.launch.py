"""Teleoperacao do Caramelo: joystick e teclado.

MUDANCA DE 2026-09-08 — este launch NAO sobe mais o twist_mux nem o twist_relay.

POR QUE: o twist_mux e' o ARBITRO de quem comanda as rodas (joystick 99 >
teclado 90 > navegacao 80). Enquanto ele morava aqui, a navegacao tinha o seu
proprio relay (nav2_twist_relay) ligando /cmd_vel direto em
/mecanum_controller/reference, POR FORA do mux. Consequencias medidas em
2026-09-08 com o robo no chao:

  - a hierarquia nao valia: um comando manual nao sobrepunha a navegacao, porque
    os dois caminhos publicavam no MESMO topico a 100 Hz e vencia quem publicasse
    por ultimo;
  - com o teleop no ar, cada comando da navegacao chegava nas rodas DUAS vezes
    (uma por cada relay), cada uma com o seu proprio carimbo de tempo.

Agora o mux e' unico e mora com a navegacao (caramelo_navigation/launch/
navigation.launch.py), que e' quem sempre esta no ar durante uma missao. Este
launch fica so' com as FONTES DE ENTRADA, que publicam nos topicos que o mux ja
assina:

    joy_teleop  -> /joy_vel      (prioridade 99)
    teclado     -> /cmd_vel_key  (prioridade 90)

Para dirigir SEM a navegacao no ar (bancada), use standalone:=true: ai este
launch sobe o seu proprio twist_mux + twist_relay.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    caramelo_utils_pkg = get_package_share_directory('caramelo_utils')
    use_sim_time = LaunchConfiguration("use_sim_time")
    standalone = LaunchConfiguration("standalone")

    # Saida do mux -> entrada do relay. Mesmo nome usado pela navegacao.
    mux_out_topic = "/caramelo_controller/cmd_vel_unstamped"

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time", default_value="False",
        description="Use simulated time")

    standalone_arg = DeclareLaunchArgument(
        name="standalone", default_value="False",
        description=(
            "Sobe um twist_mux + twist_relay proprios, para dirigir o robo SEM a "
            "navegacao no ar. Com a navegacao rodando deixe False: ela ja sobe o "
            "mux, e um segundo mux publicando no mesmo topico traz de volta "
            "exatamente o comando duplicado que a mudanca de 2026-09-08 removeu."),
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[
            os.path.join(caramelo_utils_pkg, "config", "joy_teleop.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            os.path.join(caramelo_utils_pkg, "config", "joy_config.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    teleop_keyboard_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_keyboard",
        output="screen",
        prefix="xterm -e",
        parameters=[],
        remappings=[
            # /cmd_vel -> /cmd_vel_key: e' o topico de prioridade 90 do mux.
            # Publicar em /cmd_vel direto seria escrever POR CIMA da saida do
            # collision_monitor, que e' de onde o mux le a navegacao.
            ("/cmd_vel", "/cmd_vel_key"),
        ],
    )

    # --- so' em standalone (sem navegacao no ar) --------------------------- #
    # Sem config de locks: o lock "safety_stop" foi removido em 2026-09-08 a
    # pedido do operador (a behavior tree da missao trata desvio de obstaculo).
    # Ver caramelo_utils/config/twist_mux_locks.yaml.
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            os.path.join(caramelo_utils_pkg, "config", "twist_mux_topics.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("/cmd_vel_out", mux_out_topic)],
        condition=IfCondition(standalone),
    )

    twist_relay_node = Node(
        package="caramelo_utils",
        executable="twist_relay.py",
        name="twist_relay",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_twist_topic": mux_out_topic},
            {"output_twist_stamped_topic": "/mecanum_controller/reference"},
            {"frame_id": "base_footprint"},
        ],
        output="screen",
        condition=IfCondition(standalone),
    )

    return LaunchDescription([
        use_sim_time_arg,
        standalone_arg,
        joy_teleop,
        joy_node,
        teleop_keyboard_node,
        twist_mux_node,
        twist_relay_node,
    ])
