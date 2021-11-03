# maintained by tim.schaefer.de@gmail.com
#rosrun xacro xacro `rospack find baxter_description`/urdf/baxter.urdf.xacro > baxter2.urdf

urdf_file=$1

~/git/robotics-course/rai/bin/urdf2rai.py $urdf_file > z.1.g

sed 's/package:\/\/baxter_robot_model\/meshes\///g' z.1.g > z.2.g
sed 's/package:\/\/rethink_ee_description\/meshes\///g' z.2.g > z.3.g
sed 's/\.DAE/.ply/g' z.3.g > z.4.g
sed 's/package:\/\/baxter_description\/meshes\///g' z.4.g > z.5.g

~/git/robotics-course/rai/bin/kinEdit -file z.5.g -cleanOnly
mv z.g baxter_clean_converted.g
