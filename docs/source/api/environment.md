# Environment API

In order to test an intersection control algorithm in a new environment (for example a different simulation environment
than the provided SUMO environment, or a physical environment using physical vehicles), subclasses of
{class}`intersection_control.core.environment.Environment`,
{class}`intersection_control.core.environment.VehicleHandler` and
{class}`intersection_control.core.environment.IntersectionHandler` will need to be created, and the following methods
will need to be implemented.

Any intersection control algorithm will interface with the environment in which it operates through this API.

```{eval-rst}
.. automodule:: intersection_control.core.environment
    :members:
```