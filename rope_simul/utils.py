from . import config

def meter2pix(pos):
    Ox, Oy, scale = config.Ox, config.Oy, config.scale
    CanvPos = []
    for i, xy in enumerate(pos):
        CanvPos.append(Ox + scale * xy if i % 2 == 0 else Oy - scale * xy)
    return CanvPos


def circ(pos, rad):
    cx, cy = meter2pix(pos)
    return [cx + rad, cy + rad, cx - rad, cy - rad]
