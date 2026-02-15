from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_fleet_nodes(context, *_args, **_kwargs):
    """Build launch actions for a configurable fleet size."""
    rover_count_raw = LaunchConfiguration('rover_count').perform(context)
    try:
        rover_count = int(rover_count_raw)
    except ValueError as exc:
        raise ValueError(
            f'rover_count must be an integer, got: {rover_count_raw}'
        ) from exc

    if rover_count < 1:
        raise ValueError(f'rover_count must be >= 1, got: {rover_count}')

    rover_ids = [f'rover_{idx}' for idx in range(1, rover_count + 1)]
    rover_ids_csv = ','.join(rover_ids)

    actions = [
        Node(
            package='rover_core',
            executable='space_link_node',
            name='space_link',
            namespace='space_link',
            output='screen',
        ),
        Node(
            package='rover_core',
            executable='fleet_manager',
            name='fleet_manager',
            namespace='fleet',
            output='screen',
        ),
        Node(
            package='rover_core',
            executable='earth_node',
            name='earth_station',
            namespace='earth',
            parameters=[{'rover_ids': rover_ids_csv}],
            output='screen',
        ),
        Node(
            package='rover_core',
            executable='telemetry_monitor',
            name='telemetry_monitor',
            namespace='earth',
            output='screen',
        ),
    ]

    for rover_id in rover_ids:
        actions.append(
            Node(
                package='rover_core',
                executable='rover_node',
                name=f'{rover_id}_node',
                namespace=f'rover/{rover_id}',
                parameters=[{'rover_id': rover_id}],
                output='screen',
            )
        )

    return actions


def generate_launch_description():
    """Generate launch description for the full multi-rover stack."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'rover_count',
            default_value='3',
            description='Number of rover nodes to launch (rover_1..rover_N).',
        ),
        OpaqueFunction(function=_build_fleet_nodes),
    ])
