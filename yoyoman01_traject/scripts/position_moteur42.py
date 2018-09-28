import math

# Données des trajectoires :
import data_cycle1 # demarrage
import data_cycle # Cycle normal


M=[] #position
T=[] #temps

#Demarrage
temps1=data_cycle1.t_v1[1]-data_cycle1.t_v1[0] #Pas de temps
#print("pas de temps1 demarrage="+str(temps1))

longueur1=len(data_cycle1.t_v1)

for k in range (0,longueur1):
	L=[]
	for i in range (0,24):
		L=L+[data_cycle1.q_v1[k][i]]
	M=M+[L]
	T=T+[k*temps1]

T0=data_cycle1.t_v1[len(data_cycle1.t_v1)-1]


#Cycle normal
temps=data_cycle.t_v[1]-data_cycle.t_v[0]
#print("pas de temps normal="+str(temps))

longueur=len(data_cycle.t_v)

for k in range (0,10*longueur):
	L=[]
	if (k/longueur)%2==0:
		for i in range (0,24):
			L=L+[data_cycle.q_v[k%longueur][i]]
	else:
		for i in range (0,24):
			if i==6:
				L=L+[data_cycle.q_v[k%longueur][10]]
			elif i==7:
				L=L+[data_cycle.q_v[k%longueur][11]]
			elif i==10:
				L=L+[data_cycle.q_v[k%longueur][6]]
			elif i==11:
				L=L+[data_cycle.q_v[k%longueur][7]]
			else:
				L=L+[data_cycle.q_v[k%longueur][i]]
	M=M+[L]
	T=T+[k*temps+T0]


# Mise en forme pour la générartion du fichier yaml
# cmd : python3 position_moteur42.py > yoyoman01_motions42.yaml

print(" play_motion:")
print("   controllers: full_body_controller")
print("   motions:")
print("     home:")
print("       joints: [Rarm, Larm, RHip, LHip, Head, Neck]")
print("       points:")
print("\n")

for k in range (0,int(longueur/3)):

	#print ("       - positions: ["+str(M[30*k][11]) +", "+str(M[30*k][7])+", "+str(M[30*k][10])+", "+str(M[30*k][6])+", "+str(M[30*k][9])+", "+str(M[30*k][8])+"]")
	#print ("         time_from_start: "+str(T[30*k]))

	print("       - positions: [%.13f, %.13f, %.13f, %.13f, %.13f, %.13f]" % (M[30*k][11], M[30*k][7], M[30*k][10], M[30*k][6], M[30*k][9], M[30*k][8]))
	print("         time_from_start: %.13f" % (T[30*k]))

print("\n")
print("       meta:")
print("         name: Home")
print("         usage: demo")
print("         description: '42'")