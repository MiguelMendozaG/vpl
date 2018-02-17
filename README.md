# View Planning Library

> Consejo Nacional de Ciencia y Tecnología (CONACyT)

> Instituto Nacional de Astrofísica Óptica y Electrónica (INAOE).

> Centro de Innovación y Desarrollo Tecnológico (IPN-CIDETEC).

VPL(The acronym of view planning library) provides a platform to develop view planning algorithms and perform comparisons quickly. VPL is written in C++ and it is based on a set of well known libraries: octomap and MRPT. VPL provides the data structures to represent the space, provides visibility algorithms, implements several view planning algorithms reported in the literature and provides flexibility to link with range sensor simulators and motion planning algorithms. VPL was developed by [J. Irving Vasquez-Gomez] under New BSD license.

VPL is composed of two main modules: PartialModel and ViewPlanning. PartialModel stores the information about object that is being reconstructed and provides a set of functions to handle visibility for next best view calculation. ViewPlanning provides planning algorithms to achieve an automated reconstruction. Additional modules are included but they can be ommited during compilation: rangesimulator and nbvs planning.

![VPL Examples](https://jivasquez.files.wordpress.com/2017/05/vpl_examples2.png)
  

More information is in our [VPL paper] currently under development.

If your are using VPL in an academy work please cite (so far):

Vasquez-Gomez, J. I., Sucar, L. E., & Murrieta-Cid, R. (2017). View/state planning for three-dimensional object reconstruction under uncertainty. Autonomous Robots, 41(1), 89-109.

```
@article{vasquez2017view,
  title={View/state planning for three-dimensional object reconstruction under uncertainty},
  author={Vasquez-Gomez, J Irving and Sucar, L Enrique and Murrieta-Cid, Rafael},
  journal={Autonomous Robots},
  volume={41},
  number={1},
  pages={89--109},
  year={2017},
  publisher={Springer}
}
```
If you are having troubles with VPL, please drop me a mail: 
![VPL Examples](https://jivasquez.files.wordpress.com/2017/05/ivasquez_mail.png)

### Pre-processing the input files

- File ```cloud_generation.py``` (inside pre-processing folder) generates the point clouds needed in vpl files.
In folder ```input_dataset_folder``` the folder where Hintertoisser dataset files are stored is indicated.
And ```output_dataset_folder``` indicates the location where we will save the output point clouds.

- File ```scaling.py``` scales the input point cloud of the ground truth model. 

### Requirements

Before installing VPL you need to install the following libraries:
- boost
```
  sudo apt get install libboost-all-dev
```
- [octomap]
```
git clone https://github.com/OctoMap/octomap.git
```
- [MRPT]
I installed it from Ubuntu PPA


### Installation

1. Download and install [octomap] and [MRPT].
MRPT was installed from source not PPA
2. Clone this repo to your machine
```
git clone https://github.com/irvingvasquez/vpl
```
3. Configure the CMakeLists.txt at top file

4. Compile the library. Move to the VPL top folder and run:
```sh
mkdir build
cd build 
cmake ..
```


### Full VPL installation

we recommend that VPL will be installed in the same folder that the required libraries:

- octomap-devel

To compile the library move to the VPL top folder and run:

```sh
cd VPL
mkdir build
cd build    
cmake ..
```
`Note:if you are using your custom folder hierarchy you should modify the CMakeLists.txt in order to match the folders`



   [octomap]: <https://octomap.github.io/>
   [iniparser]: <https://github.com/ndevilla/iniparser>
   [MRPT]: <http://www.mrpt.org/>
   [J. Irving Vasquez-Gomez]: https://jivasquez.wordpress.com
   [VPL paper]: https://jivasquez.files.wordpress.com/2017/05/vas_vpl_towards.pdf
