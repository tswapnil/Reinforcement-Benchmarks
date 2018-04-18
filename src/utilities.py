from subprocess import check_output

def getPoseVel(_name):
	outputStr = check_output(["rosservice", "call", "gazebo/get_model_state", '{model_name: '+ _name +'}'])
	return parsePoseVel(outputStr)

def parsePoseVel(result):
	lineList = result.splitlines()
	lines = [line.strip() for line in lineList]
	#Position 
	pos_x = float(lines[8].split(":")[1])
	pos_y = float(lines[9].split(":")[1])
	pos_z = float(lines[10].split(":")[1])
	#Orientation
	or_x = float(lines[12].split(":")[1])
	or_y = float(lines[13].split(":")[1])
	or_z = float(lines[14].split(":")[1])
	or_w = float(lines[15].split(":")[1])
	#Twist Linear and Angular
	tw_lx = float(lines[18].split(":")[1])
	tw_ly = float(lines[19].split(":")[1])
	tw_lz = float(lines[20].split(":")[1])
	tw_ax = float(lines[22].split(":")[1])
	tw_ay = float(lines[23].split(":")[1])
	tw_az = float(lines[24].split(":")[1])
	return [pos_x,pos_y,pos_z,or_x,or_y,or_z,or_w,tw_lx,tw_ly,tw_lz,tw_ax,tw_ay,tw_az]
	#return [pos_x,pos_y,pos_z]

# Linear Pos
def setPoseVel(poseList , _name):
	outputStr = check_output(["rostopic", "pub", "-1", "/gazebo/set_model_state", "gazebo_msgs/ModelState" , '{model_name: '+_name+', pose: { position: { x: '+str(poseList[0])+', y: '+str(poseList[1])+', z: '+str(poseList[2])+' }, orientation: {x: '+str(poseList[3])+', y: '+str(poseList[4])+', z: '+str(poseList[5])+', w: '+str(poseList[6])+' } }, twist: { linear: { x: '+str(poseList[7])+', y: '+str(poseList[8])+', z: '+str(poseList[9])+' }, angular: { x: '+str(poseList[10])+', y: '+str(poseList[11])+', z: '+str(poseList[12])+'}  }, reference_frame: world }'])
	return outputStr
	


def setForce(fx,fy,fz,_name, _nameLink, start_time,duration):
	outStr = check_output(["rosservice","call","/gazebo/apply_body_wrench",'{body_name: "'+str(_name)+'::'+str(_nameLink)+'", reference_frame: "'+str(_name)+'::'+str(_nameLink)+'", wrench: { force: { x: '+str(fx)+', y: '+str(fy)+', z: '+str(fz)+' } }, start_time: '+str(start_time)+', duration: '+str(duration)+' }'])
	return outStr



# Usage
print(setPoseVel([0,0,0.5,0,0,0,0,0,0,0,0,0,0], "unit_sphere_0"))
print(setPoseVel([10,0,0,0,0,0.5,0.5,0,0,0,0,0,0], "Stop Sign_0"))
print(setForce(5,0,0,"unit_sphere_0","link",0,-1))
for i in range(100):
	print(getPoseVel("unit_sphere_0"))
	print(getPoseVel("Stop Sign_0"))
	setPoseVel([0,0,0.5,0,0,0,0,0,0,0,0,0,0], "unit_sphere_0")
	setPoseVel([10,0,0,0,0,0.5,0.5,0,0,0,0,0,0], "Stop Sign_0")


