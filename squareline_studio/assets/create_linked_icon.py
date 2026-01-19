# linked_icon.py
from PIL import Image, ImageDraw, ImageChops

def _rounded_outline_mask(w, h, radius, stroke):
    """
    Return an 'L' mask of a rounded-rectangle *outline* (donut).
    - Outer rounded rect filled (255), inner cutout cleared (0).
    """
    m = Image.new("L", (w, h), 0)
    d = ImageDraw.Draw(m)
    # Outer
    d.rounded_rectangle([0, 0, w-1, h-1], radius=radius, fill=255)
    # Inner cutout (clamped so a hole always remains)
    inset = max(1, int(stroke))
    max_inset = max(1, (min(w, h) // 2) - 1)
    inset = min(inset, max_inset)
    inner_radius = max(0, radius - inset)
    if inset * 2 < min(w, h):
        d.rounded_rectangle([inset, inset, w-1-inset, h-1-inset],
                            radius=inner_radius, fill=0)
    return m

def _rounded_inner_mask(w, h, radius, stroke):
    """
    Return an 'L' mask of the inner cutout of a rounded rectangle.
    Inner (hole) is 255, outside is 0.
    """
    m = Image.new("L", (w, h), 0)
    d = ImageDraw.Draw(m)
    inset = max(1, int(stroke))
    max_inset = max(1, (min(w, h) // 2) - 1)
    inset = min(inset, max_inset)
    inner_radius = max(0, radius - inset)
    if inset * 2 < min(w, h):
        d.rounded_rectangle([inset, inset, w-1-inset, h-1-inset],
                            radius=inner_radius, fill=255)
    return m

def _link_layer(S, link_w, link_h, stroke_px, color, angle_deg, center_xy):
    """
    Build a single rotated link as an S×S RGBA layer.
    """
    layer = Image.new("RGBA", (S, S), (0, 0, 0, 0))
    cx, cy = center_xy
    x0 = int(cx - link_w/2)
    y0 = int(cy - link_h/2)

    # Make an outline mask (donut) for the link pill
    r = int(link_h / 2)  # "pill" corner radius
    mask = _rounded_outline_mask(link_w, link_h, r, stroke_px)

    # Color tile to paste through the mask
    tile = Image.new("RGBA", (link_w, link_h), color)
    layer.paste(tile, (x0, y0), mask)

    # Rotate entire layer around its center (no expand to keep size S×S)
    layer = layer.rotate(angle_deg, resample=Image.Resampling.BICUBIC, expand=False)
    return layer

def _link_inner_hole_layer(S, link_w, link_h, stroke_px, angle_deg, center_xy):
    """
    Build an S×S 'L' mask layer representing the inner hole of a single link,
    positioned and rotated exactly like the link.
    """
    layer = Image.new("L", (S, S), 0)
    cx, cy = center_xy
    x0 = int(cx - link_w/2)
    y0 = int(cy - link_h/2)
    r = int(link_h / 2)
    hole_mask = _rounded_inner_mask(link_w, link_h, r, stroke_px)
    layer.paste(hole_mask, (x0, y0), hole_mask)
    layer = layer.rotate(angle_deg, resample=Image.Resampling.BICUBIC, expand=False)
    return layer

def render_linked_icon(size=32,
                       color=(34, 197, 94, 255),     # green-500
                       thickness_ratio=0.20,         # link stroke thickness relative to link height
                       offset_ratio=0.12,            # center offset as fraction of size
                       scale=8,                      # supersample factor for crisp edges
                       bg=(0, 0, 0, 0)):             # transparent background
    """
    Render a 'Linked' icon (two interlocking chain links) and return a PIL Image (size×size RGBA).
    """
    S = size * scale
    img = Image.new("RGBA", (S, S), bg)

    # Link geometry (chosen to avoid clipping when rotated)
    link_w = int(S * 0.70)
    link_h = int(S * 0.4)
    stroke_px = max(2, int(link_h * thickness_ratio))

    # Centers for interlock (one above-left, one below-right)
    cx = cy = S // 2
    delta = int(S * offset_ratio)
    c1 = (cx - delta, cy - delta)
    c2 = (cx + delta, cy + delta)

    # Draw bottom link first, then top to suggest interlock
    bottom = _link_layer(S, link_w, link_h, stroke_px, color, 0, c2)
    top    = _link_layer(S, link_w, link_h, stroke_px, color, 0, c1)

    img.alpha_composite(bottom)
    img.alpha_composite(top)

    # No extra hole punching — rely on each link's own alpha hole
    # Downsample for smooth edges
    return img.resize((size, size), Image.Resampling.LANCZOS)

# --- Example usage ---
if __name__ == "__main__":
    path = "linked_32.png"
    img = render_linked_icon(size=32,
                             color=(34, 197, 94, 255),
                             thickness_ratio=0.2,
                             offset_ratio=0.08)
    img.save(path)
    print("wrote:", path)
