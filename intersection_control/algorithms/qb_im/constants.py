class VehicleState:
    APPROACHING = 0
    IN_INTERSECTION = 1
    DEFAULT = 2


class VehicleMessageType:
    REQUEST = 0
    """
    This is the message a vehicle sends when it does not have a reservation
    and wishes to make one. It contains the properties of the vehicle (ID number, performance, size, etc.) as well as 
    some properties of the proposed reservation (arrival time,
    arrival velocity, type of turn, arrival lane, etc.). The message also communicates the
    vehicle’s status as an emergency vehicle (in an emergency situation). In practice, this
    would be implemented using a secure method such that normal vehicles could not
    impersonate emergency vehicles. Such methods are well understood and the details
    of the implementation are beyond the scope of this research.
    
    This message has 15 fields:
    * vehicle_id               — a unique identifier for the vehicle.
    * arrival_time             — the absolute time at which the vehicle agrees to arrive at the intersection.
    * arrival_lane             — a unique identifier for the lane in which the vehicle will be when it
                                 arrives at the intersection.
    * arrival_velocity         — the velocity at which the vehicle agrees to be traveling when
                                 it arrives at the intersection.
    * vehicle_length           — the length of the vehicle.
    * vehicle_width            — the width of the vehicle.
    """

    CHANGE_REQUEST = 1
    """
    This is the message a vehicle sends when it has a reservation,
    but would like to switch to a different set of parameters. If the new parameters are
    not acceptable to the intersection, the vehicle may keep its old reservation. It is
    identical to the request message, except that it includes a unique reservation ID for
    the reservation the vehicle currently has.
    
    This message is identical to the Request message, except for one added field:
    * reservation_id — an identifier for the reservation to be changed.
    """

    CANCEL = 2
    """
    This is the message a vehicle sends when it no longer desires its current
    reservation.
    
    It has 2 fields:
    * vehicle_id     — a unique identifier for the vehicle.
    * reservation_id — an identifier for the reservation to be cancelled.
    """

    DONE = 3
    """
    This message is sent when the vehicle has completed its traversal of the
    intersection. While it communicates the same information as the Cancel message,
    there may be behavior tied to the Cancel message which should not occur when a vehicle 
    successfully completes the trip across the intersection. Additionally, this message
    could be extended in order to communicate statistics for each vehicle, which could
    then be recorded in order to analyze the performance of the intersection manager.
    This message can be used to collect statistics for each vehicle, which can be recorded
    in order to analyze and improve the performance of the intersection manager.
    
    It has 2 fields:
    * vehicle_id     — a unique identifier for the vehicle.
    * reservation_id — an identifier for the reservation that was just completed.
    """


class IMMessageType:
    CONFIRM = 0
    """
    This message is a response to a vehicle’s Request (or Change-Request)
    message. It does not always mean that the parameters transmitted by the vehicle are
    acceptable. It could, for example, contain a counter-offer by the intersection. The
    reservation parameters in this message are implicitly accepted by the vehicle, and
    must be explicitly cancelled if the driver agent of the vehicle does not approve. Note
    that this is safe even with faulty communication—the worst that can happen is that
    the intersection reserves space that does not get used. Included in the message are
    acceleration constraints determined by the intersection. This is just a list of rates and
    durations. How the list is created depends on the intersection manager. However, the
    vehicle’s safety must be guaranteed if it adheres to the list.
    
    This message has 7 fields:
    * reservation_id   — a unique identifier for the reservation just created.
    * arrival_time     — the absolute time at which the vehicle is expected to arrive.
    * early_error      — the tolerable error (early) in arrival time for the vehicle.
    * late_error       — the tolerable error (late) in arrival time for the vehicle. Note that
                         the intersection manager must assume that the car could arrive and traverse the
                         intersection at any time within the resulting bounds
    * arrival_lane     — a unique identifier for the lane in which the vehicle should be when
                         it arrives at the intersection.
    * arrival_velocity — the velocity at which the vehicle is expected to be traveling
                         when it arrives at the intersection. A negative number signifies that any velocity
                         is acceptable.
    * accelerations    — a run-length encoded description of the expected acceleration of
                         the vehicle as it travels through the intersection. Here, a run-length encoded description is a 
                         sequence of order pairs of acceleration and duration—starting
                         with the instant the vehicle enters the intersection, it should maintain each
                         acceleration for the duration with which it is paired. If the sequence is empty,
                         any accelerations are acceptable.
    """

    REJECT = 1
    """
    By sending this message, an intersection can inform a vehicle that the
    parameters sent in the latest Request (or Change-Request) were not acceptable,
    and that the intersection either could not or did not want to make a counter-offer.
    This message also indicates whether or not the rejection was because the reservation
    manager requires the vehicle to stop at the intersection before entering. This lets the
    driver agent know that it should not attempt any more reservations until it reaches
    the intersection.
    This message has 1 field:
    
    * timeout — the time at which the intersection manager will once again begin considering requests
                from this vehicle
    """

    ACKNOWLEDGE = 2
    """
    This message acknowledges the receipt of a Cancel or Done
    message.
    
    It has 1 field:
    * reservation_id — a unique identifier for the reservation just cancelled or completed.
    """

    EMERGENCY_STOP = 3
    """
    This message is only sent when the intersection manager has
    determined that a collision or similar problem has occurred in the intersection. This
    message informs the receiving driver agent that no further reservation requests will
    be granted, and if possible, the vehicle should attempt to stop instead of entering the
    intersection, even if it has a reservation. This message has no fields, as it only communicates a
    single bit of information.
    """