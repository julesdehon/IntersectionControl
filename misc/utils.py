import os

# Simple traffic light phase definition for single_intersection sumo network
SINGLE_INTERSECTION_TL_PHASES = [
    # ((Green, Yellow, Red), Duration)
    (({"NE", "NS", "NW"}, set(), {"EN", "ES", "EW", "SN", "SE", "SW", "WN", "WE", "WS"}), 10),
    ((set(), {"NE", "NS", "NW"}, {"EN", "ES", "EW", "SN", "SE", "SW", "WN", "WE", "WS"}), 2),
    (({"EN", "ES", "EW"}, set(), {"NE", "NS", "NW", "SN", "SE", "SW", "WN", "WE", "WS"}), 10),
    ((set(), {"EN", "ES", "EW"}, {"NE", "NS", "NW", "SN", "SE", "SW", "WN", "WE", "WS"}), 2),
    (({"SN", "SE", "SW"}, set(), {"NE", "NS", "NW", "EN", "ES", "EW", "WN", "WE", "WS"}), 10),
    ((set(), {"SN", "SE", "SW"}, {"NE", "NS", "NW", "EN", "ES", "EW", "WN", "WE", "WS"}), 2),
    (({"WN", "WE", "WS"}, set(), {"NE", "NS", "NW", "EN", "ES", "EW", "SN", "SE", "SW"}), 10),
    ((set(), {"WN", "WE", "WS"}, {"NE", "NS", "NW", "EN", "ES", "EW", "SN", "SE", "SW"}), 2)
]

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
