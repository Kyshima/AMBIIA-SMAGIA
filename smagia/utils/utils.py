def get_max_potency_jid(robots_dict):
    available_robots = {jid: info for jid, info in robots_dict.items() if info['availability']}

    if not available_robots:
        return None

    max_potency_jid = max(available_robots, key=lambda jid: available_robots[jid]['potency'])
    return max_potency_jid


def jid_to_string(jid):
    if jid.resource:
        return f"{jid.localpart}@{jid.domain}/{jid.resource}"
    else:
        return f"{jid.localpart}@{jid.domain}"
