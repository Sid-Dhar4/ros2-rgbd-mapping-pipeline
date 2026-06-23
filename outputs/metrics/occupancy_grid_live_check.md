# Occupancy Grid Live Verification

Live verification for the navigation-relevant occupancy-grid bridge.

## Pipeline

```text
/map/cloud -> obstacle-height filter -> /map/occupancy_grid
```

## Verified ROS topic

| Item | Observed value |
|---|---:|
| Topic | `/map/occupancy_grid` |
| Message type | `nav_msgs/msg/OccupancyGrid` |
| Publisher count | 1 |
| Frame ID | `world` |
| Resolution | 0.05 m/cell |
| Width | 120 cells |
| Height | 120 cells |
| Origin X | -3.0 m |
| Origin Y | -3.0 m |
| Publish rate | ~1.666 Hz |

## Notes

- The occupancy grid is generated from the accumulated RGB-D point-cloud map on `/map/cloud`.
- Points are filtered by obstacle-height band before projection into XY grid cells.
- The result is a costmap-style `nav_msgs/OccupancyGrid`, not a full Nav2 navigation stack.
- The node shuts down cleanly on Ctrl+C.
