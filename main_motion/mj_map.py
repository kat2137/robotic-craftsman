MJ_MAP = [
    {"link": "base",         "body": "base1",               "joint": None,               "act": None,             "driven": False, "low": None,  "high": None},
    {"link": "elbow",        "body": "elbow_main1",         "joint": "elbow_pitch",      "act": "a_elbow_pitch",  "driven": False, "low": -0.30, "high": 0.30},
    {"link": "wrist rotate", "body": "top_rotation-wheel",  "joint": "forearm_roll",     "act": "a_forearm_roll", "driven": False, "low": 2.14, "high": 3.74},
    {"link": "wrist tilt",   "body": "wrist",               "joint": "wrist_tilt",       "act": "a_wrist_tilt",   "driven": False, "low": -0.8, "high": 0.9},

    {"link": "index mcp",    "body": "index_pip",           "joint": "index_mcp_joint",  "act": "a_index_mcp",    "driven": False, "low": 1.00, "high": 0.1},
    {"link": "index ip",     "body": "index_mip",           "joint": "index_pip_joint",  "act": None,             "driven": True,  "low": None,  "high": None},
    {"link": "index dip",    "body": "index_tip",           "joint": "index_mip_joint",  "act": None,             "driven": True,  "low": None,  "high": None},
    {"link": "index tip",    "body": None,                  "joint": None,               "act": None,             "driven": False, "low": None,  "high": None},

    {"link": "thumb add",    "body": "thumb_mcp",           "joint": "thumb_mcp_joint",  "act": "a_thumb_mcp",    "driven": False, "low": 0.8, "high": -0.6},
    {"link": "thumb mcp",    "body": "thumb_pip",           "joint": "thumb_pip_joint",  "act": "a_thumb_pip",    "driven": False, "low": 1.00, "high": 0.1},
    {"link": "thumb ip",     "body": "thumb_tip",           "joint": "thumb_tip_joint",  "act": None,             "driven": True,  "low": None,  "high": None},
    {"link": "thumb tip",    "body": None,                  "joint": None,               "act": None,             "driven": False, "low": None,  "high": None},
]