from PIL import Image

img = Image.open("/home/aksc/WORK2-2026/src/planning/planning/mapa_teste.pgm")

resolution = 0.05
origin_x = -1.807
origin_y = -1.096

print(img.size)

largura, altura = img.size

print(largura, altura)

def pixel_for_coord(u, v):
    real_x = origin_x + (u * resolution)
    real_y = origin_y + (v * resolution)
    return real_x, real_y


