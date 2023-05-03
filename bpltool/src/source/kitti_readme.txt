#####################################################################################
seq     dataRaw                    start   end         seqSize   rawSize   belong	      size    baiduyun
00      2011_10_03_drive_0027      000000  004540      4541                Residential    17.6G    Yes
01      2011_10_03_drive_0042      000000  001100      1101      1176      Road            4.5G    Yes
02      2011_10_03_drive_0034      000000  004660      4661                Residential    18.0G    Yes
03      2011_09_26_drive_0067      000000  000800      801                 
04      2011_09_30_drive_0016      000000  000270      271                 Road            1.1G    Yes
05      2011_09_30_drive_0018      000000  002760      2761                Residential    10.7G    Yes
06      2011_09_30_drive_0020      000000  001100      1101                Residential     4.3G    Yes
07      2011_09_30_drive_0027      000000  001100      1101                Residential     4.3G    Yes
08      2011_09_30_drive_0028      001100  005170      4071      5184      Residential    20.0G    Yes
09      2011_09_30_drive_0033      000000  001590      1591                Residential     6.2G    Yes
10      2011_09_30_drive_0034      000000  001200      1201                Residential     4.8G    Yes


####################################################################################
03没找到资源，甚至在kitti官网上也没有标出03的下载地址

####################################################################################

the folder tree:
----01
    ----label
        --*.label
    ----oxts
        ----data
            --*.txt
        --dataformat.txt
        --timestamps.txt
    ----velodyne
    	--*.bin
    --calib.txt
    --poses.txt
    --times.txt
    --timestampsRaw.txt

The timestampsRaw.txt file come from raw data  timestamps.txt

###################################################################


###################################################################
The PCL Struct

~~~C/C++
struct VelodynePointXYZILR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint32_t label;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZILR,
                                   (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
                                           (uint16_t, label, label) (uint16_t, ring, ring) (float, time, time)
)
~~~C/C++

###################################################################

###################################################################

The IMU perform 9-axis IMU orientation-linearacc-anglerate, the orientation comfrom INS device exactly the oxts RPY

###################################################################

The label means:
0 : "unlabeled"
1 : "outlier"
10: "car"
11: "bicycle"
13: "bus"
15: "motorcycle"
16: "on-rails"
18: "truck"
20: "other-vehicle"
30: "person"
31: "bicyclist"
32: "motorcyclist"
40: "road"
44: "parking"
48: "sidewalk"
49: "other-ground"
50: "building"
51: "fence"
52: "other-structure"
60: "lane-marking"
70: "vegetation"
71: "trunk"
72: "terrain"
80: "pole"
81: "traffic-sign"
99: "other-object"
252: "moving-car"
253: "moving-bicyclist"
254: "moving-person"
255: "moving-motorcyclist"
256: "moving-on-rails"
257: "moving-bus"
258: "moving-truck"
259: "moving-other-vehicle"
