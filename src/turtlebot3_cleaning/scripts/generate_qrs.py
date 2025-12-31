#!/usr/bin/env python3
import os
import qrcode

# Rooms and payloads
rooms = {
    "LIVINGROOM": "ROOM=LIVINGROOM",
    "KITCHEN": "ROOM=KITCHEN",
    "BEDROOM": "ROOM=BEDROOM",
    "CORRIDOR": "ROOM=CORRIDOR"
}

# Output directory
output_dir = os.path.join(os.path.dirname(__file__), "../models/materials/textures")
os.makedirs(output_dir, exist_ok=True)

for room, payload in rooms.items():
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_L,
        box_size=10,
        border=4,
    )
    qr.add_data(payload)
    qr.make(fit=True)

    img = qr.make_image(fill_color="black", back_color="white")
    filename = f"qr_{room.lower()}.png"
    img.save(os.path.join(output_dir, filename))
    print(f"Generated {filename}")
