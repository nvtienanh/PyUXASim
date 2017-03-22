from Link import Link

'''(self, Name, Id, Mass, Sister, Child)'''
# The list of biped robot link
Links = []

BODY = uLink('BODY', 1, 3.5, 0, 2)
Links.append(BODY)

RLEG_J0 = uLink('RLEG_J0', 2, 0.5, 8, 3)
Links.append(RLEG_J0)

RLEG_J1 = uLink('RLEG_J1', 3, 0.5, 0, 4)
Links.append(RLEG_J1)

RLEG_J2 = uLink('RLEG_J2', 4, 0.5, 0, 5)
Links.append(RLEG_J2)

RLEG_J3 = uLink('RLEG_J3', 5, 0.5, 0, 6)
Links.append(RLEG_J3)

RLEG_J4 = uLink('RLEG_J4', 6, 0.5, 0, 7)
Links.append(RLEG_J4)

RLEG_J5 = uLink('RLEG_J5', 7, 0.5, 0, 0)
Links.append(RLEG_J5)

LLEG_J0 = uLink('LLEG_J0', 8, 0.5, 0, 9)
Links.append(LLEG_J0)

LLEG_J1 = uLink('LLEG_J1', 9, 0.5, 0, 10)
Links.append(RLEG_J1)

LLEG_J2 = uLink('LLEG_J2', 10, 0.5, 0, 11)
Links.append(LLEG_J2)

LLEG_J3 = uLink('LLEG_J3', 11, 0.5, 0, 12)
Links.append(LLEG_J3)

LLEG_J4 = uLink('LLEG_J4', 12, 0.5, 0, 13)
Links.append(LLEG_J4)

LLEG_J5 = uLink('LLEG_J5', 13, 0.5, 0, 0)
Links.append(LLEG_J5)

print(len(Links))