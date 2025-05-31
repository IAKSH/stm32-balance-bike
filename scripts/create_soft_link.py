import os

firmwares = []
firmwares += [
    name for name in os.listdir("firmware")
    if os.path.isdir(os.path.join("firmware", name)) and name not in firmwares
]

for firmware in firmwares:
    link_path = f"firmware/{firmware}/libs"
    target_path = "..\..\libs"
    os.makedirs(os.path.dirname(link_path), exist_ok=True)
    if not os.path.exists(link_path):
        os.symlink(target_path, link_path)