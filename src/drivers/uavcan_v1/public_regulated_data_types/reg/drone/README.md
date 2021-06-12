# Drone

This namespace contains the application-specific regulated DSDL namespace for aerial vehicles.
This is the core piece of the [DS-015 UAVCAN Drone Standard](https://github.com/Dronecode/SIG-UAVCAN-Drone).
If you have any questions, feel free to bring them to the [UAVCAN Forum](https://forum.uavcan.org/c/sig/drone-sig/17).

This namespace contains the following nested namespaces:

- `physics` -- abstract physical processes and states in the system.
- `service` -- narrowly specialized types for common device classes.

## Service-oriented architecture

The user of this standard is advised to get familiar with the basic principles of service-oriented design.
As a quick primer, [read Wikipedia](https://en.wikipedia.org/wiki/Service-oriented_architecture).
For a more in-depth review, please read [The UAVCAN Guide](https://uavcan.org/guide).

The value of this or any other interoperability standard is in enabling compatible, composable, and extensible
complex systems.
A design that does not put these principles above the resource utilization concerns risks defeating the purpose
of the standard.
Hence, when designing a new network service, put the clarity of your models first, then think about their performance
implications.

In service-oriented architectures, deciding what capabilities *not* to provide is just as important as
deciding what to include.
Imagine that in an aircraft you have an air data computer publishing the current airspeed.
The air data computer is aware of the placement of the pitot probe and the local effects that may
influence its readings, so correcting for these effects is the task of the air data computer itself.
Therefore, the published airspeed should be the calibrated airspeed (CAS) rather than the raw
indicated airspeed (IAS).
One might be tempted to enrich the service by providing the IAS alongside with CAS for the benefit of
the service consumer and for enhanced flexibility.
While easy to accomplish, it would be a design mistake because it might incentivize service consumers to
depend on the irrelevant implementation details of the service,
resulting in fragile architectures that are also hard to understand, maintain, and verify.
Encapsulation and interface segregation are just as important here as in software design.

Composability is desirable for high-integrity systems also because it facilitates subsystem isolation for
testing and confines changes to a smaller number of subsystems,
thus reducing the costs of validation and verification.

These design principles may constitute a departure from the practices commonly accepted in legacy systems,
which is necessitated by the increasing complexity of modern intravehicular software.

## Typical applications

The definitions in the `service` namespace contain descriptions of some common use cases
that can be addressed with this standard.
Adopters are expected to mix and match various components to create new network services that were not originally
envisioned by the authors of this standard.
This is possible while retaining full vendor-agnostic compatibility thanks to the service composability capabilities
described earlier.

A real hardware node would typically implement multiple services concurrently.
For example, a COTS (commercial off-the-shelf) electric drive may realistically implement the following:

- Naturally, the ESC service.
- The servo service for generality.
- Acoustic feedback by subscribing to `reg.drone.physics.acoustics.Note`.
- Visual feedback via the LED by subscribing to `reg.drone.physics.optics.HighColor`.

Another service that is interested in tracking the state of, say, a propeller drive
(say, for thrust estimation) would not need to concern itself with the ESC service at all.
Instead, it would simply subscribe to the generalized subject of type
`reg.drone.physics.dynamics.rotation.PlanarTs` published by the unit that drives the propeller
and extract its business-level information from that while being unaware of the specifics of the drive
(the propeller drive may be changed from an electric motor to a turboprop engine without affecting the
thrust estimation service).

As another example, a flight control unit would not need to depend on the specifics of a GNSS positioning
service to obtain the location of the vehicle.
Instead, it would subscribe to a generic subject of a highly abstract type that models the location of
the vehicle in space (along with other related information such as time and pose),
which may as well be published by a mocap rig.

## Bandwidth utilization

Being forward-looking, this design is optimized for transports that offer
the data rates of at least ~4 Mbps and the MTU of at least ~64 bytes.
It is expected that in the foreseeable future all new applications will be leveraging transports whose
data transfer capability is at this level or higher
(this includes, for example, UAVCAN/CAN over CAN FD, UAVCAN/UDP over Ethernet, UAVCAN/serial over RS-422 or USB, etc).

Applications relying on Classic CAN (maximum data rate ca. 1 Mbps, MTU 8 bytes) can still deploy these network services,
but the designer needs to be aware that most transfers will be multi-frame transfers and the resulting bus utilization
may be comparatively high.
To gauge the worst case bus utilization in a particular application, use the
[Classic CAN bandwidth estimation spreadsheet](https://docs.google.com/spreadsheets/d/1xSBcnnqbHBEZfFg4cqiS1weXHwX3X0MFWpW1WcEBIds/edit#gid=0)
(create a private copy and edit that).
Multi-frame transfers are not expected to cause performance issues because the official
UAVCAN implementation libraries are optimized for handling multi-frame transfers efficiently.

## Conventions

### Network service design conventions

- All physical quantities except error variance should be represented as `float32` by default.
  Error variance and covariance matrices should use `float16` by default.

- Covariance matrices should be represented as their upper-right triangles using the matrix packing rules
  defined in the Specification.

- Types with (co)variance should be suffixed `Var`; types with timestamp should be suffixed `Ts`;
  types with both should be suffixed `VarTs`.
  The timestamp field, if present, should be the first one;
  error (co)variance information should follow the data field(s) it relates to.

### Port naming conventions

In UAVCAN, the name of a port (i.e., subject or RPC-service) defines the names of related registers
as described in the documentation for the standard RPC-service `uavcan.register.Access`.
For instance, a node that publishes to the subject named `measurement` would have registers named
`uavcan.pub.measurement.id` and `uavcan.pub.measurement.type` (among others).

> N.B.: Contrary to other protocols, in UAVCAN, the name of a port is a node-local property that does not affect
  network exchanges over that port.
  This means that nodes can publish/subscribe to a port even if they name it differently
  as long as they are configured to use the same numerical port-ID.
  The details are given in the UAVCAN Specification.

Network service specifications given here under the `service` namespace provide the recommended names for
every defined port.
For example, the smart battery network service specification defines subjects named `status` and `parameters`.

Using the suggested names in practical implementations directly is not always possible because nodes that
implement different network services (or multiple instances of the same service) would see naming conflicts
(e.g., many services define a subject named `status`).
Hence, implementations are advised to use the recommended port names with a prefix such that ports that
relate to the same instance of a network service share the same prefix.

Imagine a node that implements two smart battery services (primary and secondary)
and a servo service (suppose we call it the main drive);
then it might have the following registers (among others):

    uavcan.pub.battery.primary.source.id
    uavcan.pub.battery.primary.status.id
    uavcan.pub.battery.primary.parameters.id
    uavcan.pub.battery.secondary.source.id
    uavcan.pub.battery.secondary.status.id
    uavcan.pub.battery.secondary.parameters.id
    uavcan.sub.main_drive.setpoint.id
    uavcan.sub.main_drive.readiness.id
    uavcan.pub.main_drive.feedback.id
    uavcan.pub.main_drive.status.id
    uavcan.pub.main_drive.power.id
    uavcan.pub.main_drive.dynamics.id

By virtue of sharing common prefixes, the registers clearly define three network services:

- `battery.primary`
- `battery.secondary`
- `main_drive`

The convention can be described in UML notation as follows:

    +-----------------------------------+
    |   Network service specification   |   E.g., the smart battery network service
    +-----------------------------------+   defined under reg.drone.service.battery
                    △ 0..*
                    ┆
                    ┆ implements
                    ┆
    +-----------------------------------+   Example group "battery.main":
    |        Prefixed port group        |   - battery.main.source
    +-----------------------------------+   - battery.main.status
                    │                       - battery.main.parameters
                    │ has
                    │
                    ♢ 1..*
    +-----------------------------------+
    |             Port                  |
    |     (subject or RPC-service)      |
    +-----------------------------------+

Following this convention is highly recommended as it aids one's understanding of the node's functional capabilities
and may enable some systems to implement automatic assignment of port identifiers.

### Behavioral conventions

Publishers of measurements or estimates should apply low-pass filtering to avoid frequency aliasing.
