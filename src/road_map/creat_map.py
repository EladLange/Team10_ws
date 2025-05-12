import numpy as np
from PIL import Image, ImageDraw
import csv
import yaml

# 🔧 הגדרות
resolution = 0.05  # מטר לפיקסל
map_size_m = 200.0  # גודל המפה במטרים (ריבוע 20x20)
map_size_px = int(map_size_m / resolution)

# 🗺 יצירת תמונה שחורה (שכל האזורים תפוסים)
img = Image.new('L', (map_size_px, map_size_px), color=0)  # 0 = שחור = תפוס
draw = ImageDraw.Draw(img)

# 🌍 המרה מקואורדינטות עולם (x,y) לפיקסלים
def world_to_map(x, y):
    mx = int((x + map_size_m / 2.0) / resolution)
    my = int((map_size_m / 2.0 - y) / resolution)
    return mx, my

# 📥 קריאה מקובץ CSV
with open("/home/yonatan/Desktop/Team10_ws/src/road_map/road_points.csv", newline='') as csvfile:
    reader = csv.DictReader(csvfile)  # קובץ עם כותרות: x,y,width
    for row in reader:
        x = float(row['x'])
        y = float(row['y'])
        width = float(row['width'])
        
        mx, my = world_to_map(x, y)
        radius_px = int((width / 2.0) / resolution)

        # ציור עיגול לבן מלא סביב הנקודה
        left_up = (mx - radius_px, my - radius_px)
        right_down = (mx + radius_px, my + radius_px)
        draw.ellipse([left_up, right_down], fill=255)  # 255 = לבן = מותר

# 💾 שמירת תמונת המפה
img.save("free_area_map.pgm")

# 💾 קובץ YAML המתאר את המפה עבור ROS
map_yaml = {
    "image": "free_area_map.pgm",
    "resolution": resolution,
    "origin": [-map_size_m / 2.0, -map_size_m / 2.0, 0.0],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.2,
}

with open("free_area_map.yaml", "w") as f:
    yaml.dump(map_yaml, f)

print("✅ Map and YAML created with filled free areas.")
