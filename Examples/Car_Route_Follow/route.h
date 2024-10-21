#ifndef ROUTE_H
#define ROUTE_H

typedef struct _waypoint{
  float distance_m;
  int drive_power;
  float heading_angle;
} route_waypoint;

route_waypoint route[] =
{
  {1.0, 220, 0.0},
  {1.0, 220, 90.0},
  {1.0, 220, 180.0},
  {1.0, 220, 270.0},
  {0.0, 0, 0.0},
};

uint8_t route_n_waypoints = sizeof(route) / sizeof(route_waypoint);
#endif /* ROUTE_H */