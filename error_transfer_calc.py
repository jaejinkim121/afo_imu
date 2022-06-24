import math


DEG2RAD = math.pi/180

foot = 0.25
shank = 0.5
thigh = 0.5

a1 = 45 * DEG2RAD
a2 = math.pi / 2 - a1
a3 = 10 * DEG2RAD
b2 = 30 * DEG2RAD
b3 = 30 * DEG2RAD

sigma_length = 0.03
sigma_angle = 5 * DEG2RAD

dzdl1 = 1 - math.cos(a1)
dzdl2 = math.sin(a2) + math.sin(b2)
dzdl3 = math.sin(a3) + math.sin(b3)

dzda1 = foot * math.sin(a1)
dzda2 = shank * math.cos(a2)
dzda3 = thigh * math.cos(a3)

dzdb2 = shank * math.cos(b2)
dzdb3 = thigh * math.cos(b3)

sigma_z = math.sqrt(
    sigma_length**2 * (dzdl1**2 + dzdl2**2 + dzdl3**2) + sigma_angle**2 * (dzda1**2 + dzda2**2 + dzda3**2 + dzdb2**2 + dzdb3**2)
)

print(sigma_z)

