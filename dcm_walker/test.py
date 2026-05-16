a = [1, 2, 3, 4, 5, 6, 7]
b = [[None]*2, a[2:]]

print(b)
import pinocchio as pin

identity = pin.SE3.Identity()