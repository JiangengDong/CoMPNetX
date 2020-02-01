
# coding: utf-8

# In[34]:

from openravepy import *
from numpy import *
from str2num import *
from rodrigues import *
from TransformMatrix import *
from TSR import *
import time
import sys
import math
import copy
import pickle
import os

#####################



def setup_init_sc(orEnv,targObj,obj_names,e_no,s_no,esc_dict,cabinets):
	env_no="env_"+str(e_no)
	sc_no="s_"+str(s_no)
	env_info=esc_dict[env_no]["initial"]
	#door
	cabinets.SetActiveDOFs([3])
	door_start=env_info["door"]["door_start"]
	door_end=env_info["door"]["door_end"]
	cabinets.SetActiveDOFValues([door_start])
	#bin
	trash=env_info["recyclingbin"]
	orEnv.Add(targObj[1])
	T0_object=trash["T0_w2"]	
	targObj[1].SetTransform(array(T0_object[0:3][:,0:4]))

	#mugblack
	name="mugblack"
	idx=obj_names.index(name)
	mug=env_info[name]
	orEnv.Add(targObj[idx])
	T0_object=mug["T0_w0"]	
	targObj[idx].SetTransform(array(T0_object[0:3][:,0:4]))

	#mugred
	name="plasticmug"
	idx=obj_names.index(name)
	mug=env_info[name]
	orEnv.Add(targObj[idx])
	T0_object=mug["T0_w0"]	
	targObj[idx].SetTransform(array(T0_object[0:3][:,0:4]))
	names=["tray","juice","fuze_bottle","coke_can","pitcher"]
	scene=esc_dict[env_no][sc_no]
	for i in range(0,len(names)):
		idx=obj_names.index(names[i])
		orEnv.Add(targObj[idx])
		if "T0_w2" in scene[names[i]].keys(): 
			T0_object=scene[names[i]]["T0_w2"]
		else:
			T0_object=scene[names[i]]["T0_w"]	
		targObj[idx].SetTransform(array(T0_object[0:3][:,0:4]))

####################

urdf_path = "/home/ahmed/comps-code/baxter_openrave/catkin_ws/src/baxter_common/baxter_description/urdf/baxter_sym.urdf"
srdf_path = "/home/ahmed/comps-code/baxter_openrave/catkin_ws/src/baxter_common/baxter_description/urdf/baxter_new.srdf"

orEnv = Environment()
orEnv.SetViewer('qtcoin')

orEnv.Reset()
module = RaveCreateModule(orEnv, 'urdf')

orEnv.SetDebugLevel(DebugLevel.Info)
colchecker = RaveCreateCollisionChecker(orEnv,'ode')
orEnv.SetCollisionChecker(colchecker)

#### tables & objects placement #####
tables=[]
table1 = RaveCreateKinBody(orEnv,'')
table1.SetName('table1')
table1.InitFromBoxes(numpy.array([[1.0, 0.4509, 0.5256, 0.2794, 0.9017, 0.5256]]),True)
orEnv.Add(table1,True)
table2 = RaveCreateKinBody(orEnv,'')
table2.SetName('table2')
table2.InitFromBoxes(numpy.array([[0.3777, -0.7303, 0.5256, 0.9017, 0.2794, 0.5256]]),True)
orEnv.Add(table2,True)
tables.append(table1)
tables.append(table2)

cabinets = orEnv.ReadRobotURI('../../ormodels/objects/furniture/my_cabinets.robot.xml')
orEnv.Add(cabinets,True)
cab_rot = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]
T0_cab = MakeTransform(mat(cab_rot),mat([0.85, 1.28, 0]).T)
cabinets.SetTransform(array(T0_cab[0:3][:,0:4]))


obj_names=["tray","recyclingbin","juice","fuze_bottle","coke_can","mugblack","plasticmug","pitcher"]
targobject=[]
targobject.append(orEnv.ReadKinBodyXMLFile('../../ormodels/objects/household/tray.kinbody.xml'))
targobject.append(orEnv.ReadKinBodyXMLFile('../../ormodels/objects/household/recyclingbin.kinbody.xml'))
targobject.append(orEnv.ReadKinBodyXMLFile('../../ormodels/objects/household/juice_bottle.kinbody.xml'))
targobject.append(orEnv.ReadKinBodyXMLFile('../../ormodels/objects/household/fuze_bottle.kinbody.xml'))
targobject.append(orEnv.ReadKinBodyXMLFile('../../ormodels/objects/household/coke_can.kinbody.xml')) 
targobject.append(orEnv.ReadKinBodyXMLFile('../../ormodels/objects/household/mugblack.kinbody.xml')) 
targobject.append(orEnv.ReadKinBodyXMLFile('../../ormodels/objects/household/mug2.kinbody.xml'))
targobject.append(orEnv.ReadKinBodyXMLFile('../../ormodels/objects/household/pitcher3.kinbody.xml'))



########## load robot and place ###
name = module.SendCommand('LoadURI {} {}'.format(urdf_path, srdf_path))
robot = orEnv.GetRobot(name)

T0_baxter = MakeTransform(mat(eye(3)),mat([0.20, 0.15, 0.9242]).T)
robot.SetTransform(array(T0_baxter[0:3][:,0:4]))

#create problem instances
probs_manip = RaveCreateProblem(orEnv,'Manipulation')
orEnv.LoadProblem(probs_manip,robot.GetName())

probs_cbirrt = RaveCreateProblem(orEnv,'CBiRRT')
orEnv.LoadProblem(probs_cbirrt,robot.GetName())

# define cabinet problem instance
'''probs_cbirrt2 = RaveCreateProblem(orEnv, 'CBiRRT')
orEnv.LoadProblem(probs_cbirrt2, cabinets.GetName())'''

# cabinet hinge joint
cab_joint_ind = 3

jointtm = str2num(probs_cbirrt.SendCommand('GetJointTransform name cabinets jointind %d'%cab_joint_ind))

# set initial configuration
arm0dofs = [2, 3, 4, 5, 6, 7, 8]
arm1dofs = [10, 11, 12, 13, 14, 15, 16]
activedofs = arm0dofs + arm1dofs
initdofvals = r_[-0.386, 1.321, -0.06, 0.916, -0.349, -0.734, -1.84, 0.216, 1.325, 0.173, 0.581, 0.490, -0.3883, 1.950]

robot.SetActiveDOFs(activedofs)
robot.SetActiveDOFValues(initdofvals)

# open hands
handdof = r_[(1*ones([1,2]))[0]];
robot.SetActiveDOFs([1, 9])
robot.SetActiveDOFValues(handdof)


print orEnv.CheckCollision(robot)

report = CollisionReport()


### modularize

#time.sleep(30)

e_no=10
sc_no=30

esc_dict =  pickle.load(open( "esc_dict_door9_30_test4.p", "rb" ))
fpr=0
fpp=0
tpr=0
tpp=0
total_time_reach=0
total_time_pp=0
for i in range (0,9):
	for j in range(0,30):
		env_no="env_"+str(i)
		sc_no="s_"+str(j)
		rgconf=esc_dict[env_no][sc_no]["mugblack"]["rgconf"]
		esc_dict[env_no][sc_no]["initial"]["plasticmug"]["riconf"]=rgconf
### when loading files
for e in range(0, 1):
	for s in range (0,30): #30
		env_no="env_"+str(e)
		s_no="s_"+str(s)
		arm1dofs = [10, 11, 12, 13, 14, 15, 16]
		initdofvals = r_[-0.386, 1.321, -0.06, 0.916, -0.349, -0.734, -1.84]
		robot.SetActiveDOFs(arm1dofs)
		robot.SetActiveDOFValues(initdofvals)
		numTSRChainMimicDOF = 1

		setup_init_sc(orEnv,targobject,obj_names,e,s,esc_dict,cabinets)
		print("####################################################################")
		print("env_no:"+env_no+" s_no:"+s_no)
		obj_order=esc_dict[env_no][s_no]["obj_order"]
		print(obj_order)

		robot.SetActiveDOFValues(initdofvals)
		##main loop
		idx=None
		broke=None
		names1=["juice","fuze_bottle","coke_can","pitcher"]
		names2=["mugblack","plasticmug"]
		for i in range (0,len(obj_order)):
			print(obj_order[i])
			if obj_order[i] in names1:
				T0_w=esc_dict[env_no][s_no][obj_order[i]]["T0_w"]
				T0_w2=esc_dict[env_no][s_no]["initial"][obj_order[i]]["T0_w2"]
				Bw2=esc_dict[env_no][s_no]["initial"][obj_order[i]]["Bw2"]
				Tw_e=esc_dict[env_no][s_no][obj_order[i]]["Tw_e"]
				Bw=esc_dict[env_no][s_no][obj_order[i]]["Bw"]
				if obj_order[i] != "pitcher":
					Bw2=mat([-1000., 1000.,   -1000., 1000.,   -1000, 1000,   -pi, pi,   -pi, pi,   -pi, pi])
				TSRstring1 = SerializeTSR(1,'NULL',T0_w,Tw_e,Bw)
				TSRChainString1 = SerializeTSRChain(0,0,0,1,TSRstring1,'NULL',[])

				TSRstring2 = SerializeTSR(1,'NULL',T0_w2,Tw_e,Bw2)
				TSRChainString2 = SerializeTSRChain(0,1,1,1,TSRstring2,'NULL',[])

				initdofvals=esc_dict[env_no][s_no][obj_order[i]]["riconf"]
				startik=esc_dict[env_no][s_no][obj_order[i]]["rsconf"]
				goalik=esc_dict[env_no][s_no]["initial"][obj_order[i]]["rgconf"]

			elif obj_order[i]=="door":
				T0_w0=esc_dict[env_no][s_no]["initial"]["door"]["T0_w0"]
				Tw0_e=esc_dict[env_no][s_no]["initial"]["door"]["Tw0_e"]
				Bw0=esc_dict[env_no][s_no]["initial"]["door"]["Bw0"]
				door_init=esc_dict[env_no][s_no]["initial"]["door"]["door_start"]
				door_end=esc_dict[env_no][s_no]["initial"]["door"]["door_end"]

				Tw1_e=esc_dict[env_no][s_no]["initial"]["door"]["Tw1_e"]
				Bw1=esc_dict[env_no][s_no]["initial"]["door"]["Bw1"]

				TSRstring1 = SerializeTSR(1,'NULL',T0_w0,Tw0_e,Bw0)
				TSRstring2 = SerializeTSR(1,'NULL',mat(eye(4)),Tw1_e,Bw1) 
				TSRChainString = SerializeTSRChain(0,0,1,2,TSRstring1 + ' ' + TSRstring2,'cabinets',mat(cab_joint_ind))


				initdofvals=esc_dict[env_no][s_no]["initial"]["door"]["riconf"]
				startik=esc_dict[env_no][s_no]["initial"]["door"]["rsconf"]
				goalik=esc_dict[env_no][s_no]["initial"]["door"]["rgconf"]

			elif obj_order[i] in names2:
				T0_w=esc_dict[env_no][s_no]["initial"][obj_order[i]]["T0_w0"]
				T0_w2=esc_dict[env_no][s_no][obj_order[i]]["T0_w2"]
				Bw2=esc_dict[env_no][s_no][obj_order[i]]["Bw2"]
				Tw_e=esc_dict[env_no][s_no][obj_order[i]]["Tw_e"]
				Bw=esc_dict[env_no][s_no]["initial"][obj_order[i]]["Bw"] #missing

				TSRstring1 = SerializeTSR(1,'NULL',T0_w,Tw_e,Bw)
				TSRChainString1 = SerializeTSRChain(0,0,0,1,TSRstring1,'NULL',[])

				TSRstring2 = SerializeTSR(1,'NULL',T0_w2,Tw_e,Bw2)
				TSRChainString2 = SerializeTSRChain(0,1,1,1,TSRstring2,'NULL',[])

				initdofvals=esc_dict[env_no][s_no]["initial"][obj_order[i]]["riconf"]
				if obj_order[i]=="plasticmug":
					initdofvals=esc_dict[env_no][s_no][obj_order[i-1]]["rgconf"]
				startik=esc_dict[env_no][s_no]["initial"][obj_order[i]]["rsconf"]
				goalik=esc_dict[env_no][s_no][obj_order[i]]["rgconf"]

			robot.SetActiveDOFs(arm1dofs)
			robot.SetActiveDOFValues(initdofvals)
			time.sleep(0.1)



			vpath="/home/ahmed/comps-code/examples/python_baxter/test_data/seen_reps_txt4/e_"+str(0)+"_s_"+str(110)+"_coke_can_voxel.csv"
			opath="/home/ahmed/comps-code/examples/python_baxter/test_data/seen_reps_txt4/e_"+str(0)+"_s_"+str(110)+"_coke_can_ohot.csv"
			ppath="/home/ahmed/comps-code/pytorch_code/ctpnet_annotated_gpu4.pt"


			traj_name="test_traj/"+obj_order[i]+"_traj1.txt"
			if idx is not None: orEnv.Remove(targobject[idx])
			start=time.time()
			resp=probs_cbirrt.SendCommand('RunCBiRRT timelimit 300 filename %s pnetpath %s voxelpath %s onehotpath %s jointgoals %s %s'%(traj_name,ppath,vpath,opath,len(startik),
			Serialize1DMatrix(mat(startik))))
			if str(resp[0])=="1":
				fpr=fpr+1
				end=time.time()
				print("time: "+str(end-start))
				probs_cbirrt.SendCommand('traj %s'%traj_name)
				robot.WaitForController(0)
				time.sleep(0.02)
				t_time=fromfile("total_time.dat")
				total_time_reach=total_time_reach+t_time[0]
				path=fromfile("path_data.dat")
				path=path.reshape(len(path)/35,35)
				path_list=[]
				for p in range (0, len(path)):
					path_list.append(path[p][10:17].tolist())
				if obj_order[i]=="door":
					esc_dict[env_no][s_no]["initial"][obj_order[i]].update({"path_reach":path_list})
					esc_dict[env_no][s_no]["initial"][obj_order[i]].update({"time_reach":t_time})

				else:
					esc_dict[env_no][s_no][obj_order[i]].update({"path_reach":path_list})
					esc_dict[env_no][s_no][obj_order[i]].update({"time_reach":t_time})

				os.remove("path_data.dat") 
				os.remove("total_time.dat") 

			robot.SetActiveDOFs(arm1dofs)
			robot.SetActiveDOFValues(startik)
			time.sleep(0.1)

			if idx is not None: orEnv.Remove(targobject[idx])
			time.sleep(0.02) 
			traj_name="test_traj/"+obj_order[i]+"_traj2.txt"
			if obj_order[i]=="door":

				cabinets.SetActiveDOFs([3])
				cabinets.SetActiveDOFValues([door_end])

				robot.SetActiveDOFs(arm1dofs)
				robot.SetActiveDOFValues(goalik)

				resp=[]
				resp.append(1)
			else:
				grab="GrabBody name "+obj_order[i]
				probs_manip.SendCommand('setactivemanip index 1')
				resp=probs_manip.SendCommand(grab)
				time.sleep(0.02)
				print "Press return to go grab"
				start=time.time()
				resp=probs_cbirrt.SendCommand('RunCBiRRT filename %s pnetpath %s voxelpath %s onehotpath %s  psample 0.25 timelimit 300 smoothingitrs 50 jointgoals %s %s %s'%(traj_name,
                ppath,vpath,opath,len(goalik),Serialize1DMatrix(mat(goalik)),TSRChainString2))

			if str(resp[0])=="1":

				end=time.time()
				fpp=fpp+1
				print("time: "+str(end-start))
				if obj_order[i]!="door":
					probs_cbirrt.SendCommand('traj %s'%traj_name)
					path=fromfile("path_data.dat")
					t_time=fromfile("total_time.dat")
					total_time_pp=total_time_pp+t_time[0]
					path=path.reshape(len(path)/35,35)
					path_list=[]
					for p in range (0, len(path)):
						path_list.append(path[p][10:17].tolist())
					esc_dict[env_no][s_no][obj_order[i]].update({"path_pick_place":path_list})

					esc_dict[env_no][s_no][obj_order[i]].update({"time_pick_place":t_time})
					os.remove("path_data.dat")
					os.remove("total_time.dat") 
					objpath=fromfile("objpath_data.dat")
					objpath=objpath.reshape(len(objpath)/7,7)
					esc_dict[env_no][s_no][obj_order[i]].update({"obj_path":objpath})
					os.remove("objpath_data.dat") 
					time.sleep(0.02)


					robot.ReleaseAllGrabbed()
					robot.WaitForController(0)



			robot.ReleaseAllGrabbed()
			robot.WaitForController(0)
			robot.SetActiveDOFs(arm1dofs)
			robot.SetActiveDOFValues(goalik)

			tpr=tpr+1
			tpp=tpp+1

			if obj_order[i] =="mugblack" or obj_order[i] == "plasticmug" or obj_order[i] == "pitcher":
				idx=obj_names.index(obj_order[i])
				targobject[idx].SetTransform(array(T0_w2[0:3][:,0:4]))
				idx=None
			elif obj_order[i] == "door":
				idx=None
			else:
				idx=obj_names.index(obj_order[i])
				print(targobject[idx])
				print("++++++++++++++++++remove "+ obj_order[i])
				orEnv.Remove(targobject[idx])
				time.sleep(0.05)


		pickle.dump( esc_dict, open( "esc_dict_door_test4_cbirrt.p", "wb" ) )
		time.sleep(0.02)

print("totalpathspickplace:"+str(tpp))
print("feasiblepathspickplace:"+str(fpp))
print("meantimepickplace:"+str(total_time_pp/fpp))


print("totalpathsreach:"+str(tpr))
print("feasiblepathsreach:"+str(fpr))
print("meantimereach:"+str(total_time_reach/fpr))
print("totalpaths:"+str(tpr+tpp))
print("feasiblepaths:"+str(fpr+fpp))
print("meantime:"+str((total_time_reach+total_time_pp)/(fpp+fpr)))
input("yes")




