# wifi_and_battery_icons.py
# Render Wi-Fi + Battery icons (individual PNGs) or build row sprite sheets.

from PIL import Image, ImageDraw
from math import ceil
import os

# ------------------------------
# Wi-Fi ICONS
# ------------------------------
def render_wifi(level,
                size=32,
                color=(34, 197, 94, 255),
                off_color=(80, 80, 80, 255),
                span_deg=70.0,
                dot_y=0.92,
                arc_thickness_ratio=0.10,
                dot_size=0.09,
                scale=8):
    """
    Render a Wi-Fi icon (0..4 arcs lit) and return a PIL Image (RGBA, size x size).
    """
    S = size * scale
    img = Image.new("RGBA", (S, S), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)

    cx, cy = S // 2, int(S * dot_y)
    w = int(S * arc_thickness_ratio)
    dot_r = int(S * dot_size)

    start = 270.0 - (span_deg / 2.0)
    end   = 270.0 + (span_deg / 2.0)

    def draw_arc(rad, col, width):
        bbox = (cx - rad, cy - rad, cx + rad, cy + rad)
        d.arc(bbox, start=start, end=end, fill=col, width=width)

    # Draw arcs (off vs on)
    for idx in range(0, 4):
        radius = int(S * (0.16 + dot_size + dot_size + (0.16 * idx)))
        draw_arc(radius, color if level >= idx + 1 else off_color, w)

    # Bottom dot (always "on")
    d.ellipse((cx - dot_r, cy - dot_r - dot_r,
               cx + dot_r, cy + dot_r - dot_r), fill=color)

    # Supersample down for nicer edges
    return img.resize((size, size), Image.Resampling.LANCZOS)

def draw_wifi(level, size=32, color=(34,197,94,255), off_color=(80,80,80,255), path="wifi.png"):
    """Back-compat: render a single Wi-Fi icon and save it."""
    img = render_wifi(level=level, size=size, color=color, off_color=off_color)
    img.save(path)
    return path

def write_wifi_assets(levels=range(5),
                      size=32,
                      color=(34,197,94,255),
                      off_color=(80,80,80,255),
                      out_dir=".",
                      basename="wifi",
                      spritesheet=False,
                      padding=0,
                      sheet_path=None):
    """
    Generate Wi-Fi icons as individual PNGs, or as a single ROW sprite sheet.
    When spritesheet=True, output is a horizontal strip (left→right by level).
    """
    os.makedirs(out_dir, exist_ok=True)
    levels = list(levels)

    if not spritesheet:
        paths = []
        for lvl in levels:
            p = os.path.join(out_dir, f"{basename}_{lvl}_{size}.png")
            draw_wifi(level=lvl, size=size, color=color, off_color=off_color, path=p)
            paths.append(p)
        return paths, None

    # Row sprite sheet
    n = len(levels)
    sheet_w = n * size + padding * (n - 1)
    sheet_h = size
    sheet = Image.new("RGBA", (sheet_w, sheet_h), (0, 0, 0, 0))

    for i, lvl in enumerate(levels):
        tile = render_wifi(level=lvl, size=size, color=color, off_color=off_color)
        x = i * (size + padding)
        sheet.paste(tile, (x, 0), tile)

    if sheet_path is None:
        sheet_path = os.path.join(out_dir, f"{basename}_sheet_{size}_row.png")

    sheet.save(sheet_path)
    return [], sheet_path

# ------------------------------
# BATTERY ICONS
# ------------------------------
def render_battery(level,
                   width=42, height=28,
                   fill_col=(34,197,94,255),
                   track_color=(90,90,90,255),     # inner track
                   outline_color=(60,60,60,255),   # body/cap outline
                   scale=8):
    """
    Create a battery icon at any width x height.
    level: 0..100 (%)
    Returns a PIL Image (RGBA, width x height).
    """
    Sx, Sy = width * scale, height * scale
    img = Image.new("RGBA", (Sx, Sy), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)

    base = min(Sx, Sy)                # keep proportions sensible
    pad = int(base * 0.09)
    cap_w = int(base * 0.16)
    body_r = int(base * 0.14)
    stroke = max(2, int(base * 0.08))
    track_pad = int(stroke * 0.9)

    body = (pad, pad, Sx - pad - cap_w, Sy - pad)

    cap_h = int((Sy - 2 * pad) * 0.55)
    cap_y0 = (Sy - cap_h) // 2
    cap = (Sx - pad - cap_w, cap_y0, Sx - pad, cap_y0 + cap_h)

    track = (body[0] + track_pad,
             body[1] + track_pad,
             body[2] - track_pad,
             body[3] - track_pad)
    track_r = max(1, body_r - track_pad)

    # Track
    d.rounded_rectangle(track, radius=track_r, fill=track_color)

    # Fill
    lvl = max(0, min(100, int(level)))
    inner_w = max(0, track[2] - track[0])
    fill_w = int(inner_w * (lvl / 100.0))
    if fill_w > 0:
        fill_rect = (track[0], track[1], track[0] + fill_w, track[3])
        if fill_w >= track_r * 2:
            d.rounded_rectangle(fill_rect, radius=track_r, fill=fill_col)
        else:
            d.rectangle(fill_rect, fill=fill_col)

    # Outline + cap
    d.rounded_rectangle(body, radius=body_r, outline=outline_color, width=stroke)
    d.rectangle(cap, outline=outline_color, width=stroke, fill=track_color)

    return img.resize((width, height), Image.Resampling.LANCZOS)

def draw_battery(level, fill_col, path,
                 width=42, height=28,
                 track_color=(90,90,90,255),
                 outline_color=(60,60,60,255)):
    """Render a single battery icon and save it."""
    img = render_battery(level=level, width=width, height=height,
                         fill_col=fill_col,
                         track_color=track_color,
                         outline_color=outline_color)
    img.save(path)
    return path

def write_battery_assets(level_color_pairs=None,
                         width=42, height=28,
                         out_dir=".",
                         basename="battery",
                         spritesheet=False,
                         padding=0,
                         sheet_path=None,
                         track_color=(90,90,90,255),
                         outline_color=(60,60,60,255)):
    """
    Generate Battery icons from (level, color) pairs.
    When spritesheet=True, output is a single ROW sprite sheet in the order given.
    """
    os.makedirs(out_dir, exist_ok=True)

    if level_color_pairs is None:
        # Default set
        level_color_pairs = [(10, (239, 68, 68, 255)),   # RED
                             (25, (245,158, 11, 255)),   # ORANGE
                             (50, ( 34,197, 94, 255)),   # GREEN
                             (75, ( 34,197, 94, 255)),   # GREEN
                             (100,( 34,197, 94, 255))]   # GREEN

    if not spritesheet:
        paths = []
        for lvl, col in level_color_pairs:
            p = os.path.join(out_dir, f"{basename}_{lvl}.png")
            draw_battery(lvl, col, p, width=width, height=height,
                         track_color=track_color, outline_color=outline_color)
            paths.append(p)
        return paths, None

    # Row sprite sheet
    n = len(level_color_pairs)
    sheet_w = n * width + padding * (n - 1)
    sheet_h = height
    sheet = Image.new("RGBA", (sheet_w, sheet_h), (0, 0, 0, 0))

    for i, (lvl, col) in enumerate(level_color_pairs):
        tile = render_battery(lvl, width=width, height=height, fill_col=col,
                              track_color=track_color, outline_color=outline_color)
        x = i * (width + padding)
        sheet.paste(tile, (x, 0), tile)

    if sheet_path is None:
        sheet_path = os.path.join(out_dir, f"{basename}_sheet_{width}x{height}_row.png")

    sheet.save(sheet_path)
    return [], sheet_path

# ------------------------------
# Example usage
# ------------------------------
if __name__ == "__main__":
    colors = {
        "black": (0, 0, 0, 255),
        "slate": (51, 65, 85, 255),
        "blue":  (37, 99, 235, 255),
        "green": (34, 197, 94, 255),
        "red":   (239, 68, 68, 255),
        "white": (255,255,255,255),
    }

    # 1) Wi-Fi: individual icons (keeps your original flow)
    for lvl in range(5):
        draw_wifi(level=lvl, size=32, color=colors["green"], path=f"wifi_{lvl}_32.png")

    # 2) Wi-Fi: row sprite sheet (0..4 left→right)
    _, wifi_sheet = write_wifi_assets(levels=range(5),
                                      size=32,
                                      color=colors["green"],
                                      spritesheet=True,      # row mode
                                      padding=0,
                                      sheet_path="wifi_sheet_32_row.png")
    print("wrote:", wifi_sheet)

    # 3) Battery: individual icons
    RED    = (239, 68, 68, 255)
    ORANGE = (245,158, 11, 255)
    GREEN  = ( 34,197, 94, 255)

    level_color_pairs = [(10, RED), (25, ORANGE), (50, GREEN), (75, GREEN), (100, GREEN)]
    for lvl, col in level_color_pairs:
        draw_battery(lvl, col, f"battery_{lvl}.png")

    # 4) Battery: row sprite sheet (same order as level_color_pairs)
    _, bat_sheet = write_battery_assets(level_color_pairs=level_color_pairs,
                                        width=42, height=28,
                                        spritesheet=True,     # row mode
                                        padding=0,
                                        sheet_path="battery_sheet_row.png")
    print("wrote:", bat_sheet)