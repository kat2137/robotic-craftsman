from f_kinematics import LINKS

def expansion(q):
    Q = {}
    for link in LINKS:
        if link["driver"] is None:
            val = q.get(link["name"], 0.0)
            Q[link["name"]] = val
        else:
            val = q[link["driver"]]
            Q[link["name"]] = val*link["ratio"]