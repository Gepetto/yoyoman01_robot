import math

# Données des trajectoires :
import data_cycle1 # demarrage
import data_cycle # Cycle normal

M=[] #position
T=[] #temps

#Calcul pas de temps
temps1=data_cycle1.t_v1[1]-data_cycle1.t_v1[0] #Cycle demarrage
temps2=data_cycle.t_v[1]-data_cycle.t_v[0] #Cycle normal


#Taille tableau
longueur1=len(data_cycle1.t_v1)
longueur2=len(data_cycle.t_v)

#PARAMETRES ECHANTILLONAGE
NBR_REPET = 10 
#Dans la base de données le pas de temps n'est pas égale entre l'echantillon 1 et 2
TPS_ECHTILLON1 = 42 #0.02 soit 50HZ
TPS_ECHTILLON2 = 22 #0.02 soit 50HZ
LONG_ECHTILLON_1 = longueur1/TPS_ECHTILLON1
LONG_ECHTILLON_2 = (NBR_REPET *longueur2)/TPS_ECHTILLON2


#Cycle demarrage
for k in range (0,longueur1):
	L=[]
	for i in range (0,24):
		L=L+[data_cycle1.q_v1[k][i]]
	M=M+[L]
	T=T+[k*temps1]

T0=data_cycle1.t_v1[len(data_cycle1.t_v1)-1]


#Cycle normal
for k in range (0,NBR_REPET *longueur2):
	L=[]
	if (k/longueur2)%2==0:
		for i in range (0,24):
			L=L+[data_cycle.q_v[k%longueur2][i]]
	else:
		for i in range (0,24):
			if i==6:
				L=L+[data_cycle.q_v[k%longueur2][10]]
			elif i==7:
				L=L+[data_cycle.q_v[k%longueur2][11]]
			elif i==10:
				L=L+[data_cycle.q_v[k%longueur2][6]]
			elif i==11:
				L=L+[data_cycle.q_v[k%longueur2][7]]
			else:
				L=L+[data_cycle.q_v[k%longueur2][i]]
	M=M+[L]
	T=T+[k*temps2+T0]




# Mise en forme pour la générartion du fichier yaml
# cmd : python3 position_moteur42.py > yoyoman01_motions42.yaml

print(" play_motion:")
print("   controllers: full_body_controller")
print("   motions:")
print("     home:")
print("       joints: [Rarm, Larm, RHip, LHip, Head, Neck]")
print("       points:")
print("\n")


# Deux Traitements différents pour t_v1 et t_v car les pas de temps sont différents 
# Decalage de +1 pour le cycle de demarrage afin d'exclure le temps 0.00000 source de bugs
for k in range (0,int(LONG_ECHTILLON_1)):
	print("       - positions: [%.16f, %.16f, %.16f, %.16f, %.16f, %.16f]" % (M[TPS_ECHTILLON1*k+1][11], M[TPS_ECHTILLON1*k+1][7], M[TPS_ECHTILLON1*k+1][10], M[TPS_ECHTILLON1*k+1][6], M[TPS_ECHTILLON1*k+1][9], M[TPS_ECHTILLON1*k+1][8]))
	print("         *time_from_start: %.16f" % (T[TPS_ECHTILLON1*k+1]))
for k in range (0,int(LONG_ECHTILLON_2)):
	print("       - positions: [%.16f, %.16f, %.16f, %.16f, %.16f, %.16f]" % (M[longueur1+TPS_ECHTILLON2*k][11], M[longueur1+TPS_ECHTILLON2*k][7], M[longueur1+TPS_ECHTILLON2*k][10], M[longueur1+TPS_ECHTILLON2*k][6], M[longueur1+TPS_ECHTILLON2*k][9], M[longueur1+TPS_ECHTILLON2*k][8]))
	print("         time_from_start: %.16f" % (T[longueur1+TPS_ECHTILLON2*k]))

print("\n")
print("       meta:")
print("         name: Home")
print("         usage: demo")
print("         description: '42'")