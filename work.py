import matplotlib.pyplot as plt

from vehicle_model.kin_model.kin_model import KinModel

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

new_kin = KinModel(vehicle_yaml = "./vehicle.yaml", corner = 0)
new_kin.plot(new_kin.vecs, ax)

new_kin.jounce(magnitude = 1)
new_kin.plot(new_kin.vecs, ax)

new_kin.jounce(magnitude = 2)
new_kin.plot(new_kin.vecs, ax)

new_kin.jounce(magnitude = 1)

new_kin.jounce(magnitude = 0)

new_kin.jounce(magnitude = -1)
new_kin.plot(new_kin.vecs, ax)

new_kin.jounce(magnitude = -2)
new_kin.plot(new_kin.vecs, ax)

plt.show()