# IMPORT EXTERNAL ITEMS
import Docking_Functions as DF


DF.FindStation()
DF.CloseThetaOffset()
DF.GetInFront(323)
# DF.CloseThetaOffset()
# DF.GetInFront(323)
# DF.CloseThetaOffset()
DF.driveStraight()

# currenthdg = DF.GetH360()

#SH=GetSH()
# TH = DF.GetTH(326)
# colorTarget = DF.ct.colorTarget(DF.color_range)

# #DEBUG
# print("X:",colorTarget[0])
# print("hdg:",currenthdg)
# print("TH:",TH)
# while ( (((TH+4) < currenthdg ) or (currenthdg < (TH-4))) and ( (137 < colorTarget[0]) or (colorTarget[0] < 118) )   ):
#     DF.GetInFront(326)
#     DF.CloseThetaOffset()
#     colorTarget = DF.ct.colorTarget(DF.color_range)
#     currenthdg = DF.GetH360()
#     #DEBUG
#     print("X:",colorTarget[0])
#     print("hdg:",currenthdg)

