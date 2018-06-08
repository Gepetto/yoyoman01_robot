
L=["Rarm", "Larm", "RHip", "LHip", "Head", "Neck", "RKnee", "LKnee", "RFoot", "LFoot", "fixed"]
for k in range (0,11):
 for i in range (k+1,11):
   print("   <disable_collisions link1='"+L[k]+"' link2='"+L[i]+"' reason='Never' />")
