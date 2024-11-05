from PIL import Image, ImageDraw

img = Image.open("/Users/Asus/Documents/create copy/controllers/my_ground/yellow.jpg").convert("RGBA")

# Generate 4 levels of opacity
# opacity_levels = [i for i in range(256)]  # (0 = transparent, 255 = opaque)
# for i, opacity in enumerate(opacity_levels):
#     img_with_opacity = image.copy()
#     img_with_opacity.putalpha(opacity)
#     img_with_opacity.save(f"/Users/Asus/Documents/create/controllers/my_ground/dust_opacity_{i}.png")

img = img.resize((20, 20))

mask = Image.new("L", img.size, 0)
draw = ImageDraw.Draw(mask)

# Draw a white circle on the mask
width, height = img.size
draw.ellipse((0, 0, width, height), fill=255)

# Create a new image with transparency
circular_image = Image.new("RGBA", img.size, (0, 0, 0, 0))

# Paste the original image onto the new image using the circular mask
circular_image.paste(img.convert("RGBA"), (0, 0), mask)

# Save the circular image
circular_image.save("/Users/Asus/Documents/create copy/controllers/my_ground/yellow_resized_circ_20.png")
