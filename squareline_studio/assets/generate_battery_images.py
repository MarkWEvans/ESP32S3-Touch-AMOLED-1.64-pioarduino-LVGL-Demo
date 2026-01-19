from PIL import Image, ImageDraw

from PIL import Image, ImageDraw

def draw_battery(level, fill_col, path, 
                    width= 42, height = 28,
                    track_color=(90,90,90,255),   # inner track
                    outline_color=(60,60,60,255), # body/cap outline
                    ):
    """
    Create a battery icon at any width x height.
    level: 0..100 (%)
    """
    scale = 8
    Sx, Sy = width * scale, height * scale
    img = Image.new("RGBA", (Sx, Sy), (0,0,0,0))
    d = ImageDraw.Draw(img)

    base = min(Sx, Sy)                # keep proportions sensible
    pad = int(base * 0.09)
    cap_w = int(base * 0.16)
    body_r = int(base * 0.14)
    stroke = max(2, int(base * 0.08))
    track_pad = int(stroke * 0.9)

    body = (pad, pad, Sx - pad - cap_w, Sy - pad)

    cap_h = int((Sy - 2*pad) * 0.55)
    cap_y0 = (Sy - cap_h)//2
    cap = (Sx - pad - cap_w, cap_y0, Sx - pad, cap_y0 + cap_h)

    track = (body[0] + track_pad,
             body[1] + track_pad,
             body[2] - track_pad,
             body[3] - track_pad)
    track_r = max(1, body_r - track_pad)

    d.rounded_rectangle(track, radius=track_r, fill=track_color)

    lvl = max(0, min(100, int(level)))

    inner_w = max(0, track[2] - track[0])
    fill_w = int(inner_w * (lvl / 100.0))
    if fill_w > 0:
        fill_rect = (track[0], track[1], track[0] + fill_w, track[3])
        if fill_w >= track_r * 2:
            d.rounded_rectangle(fill_rect, radius=track_r, fill=fill_col)
        else:
            d.rectangle(fill_rect, fill=fill_col)

    d.rounded_rectangle(body, radius=body_r, outline=outline_color, width=stroke)
    d.rectangle(cap, outline=outline_color, width=stroke, fill=track_color)

    img.resize((width, height), Image.Resampling.LANCZOS).save(path)
    return path

RED    = (239, 68, 68, 255)
ORANGE = (245,158, 11, 255)
GREEN  = ( 34,197, 94, 255)
for lvl, color in [(10, RED), (25, ORANGE), (50, GREEN), (75, GREEN), (100, GREEN)]:
     draw_battery(lvl, color, f"battery_{lvl}.png")
