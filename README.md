# A Three Phase VSC-HVDC Average Value Model Implementation in Modelica

![alt text](https://github.com/ALSETLab/2017_ModelicaConf_VSC-HVDC_AVM_Model/blob/master/Example_Results/Dymola2018/VSC/TwoNodePowerSystem.png)

This repository contains a Modelica VSC-HVDC Average Value Model Implementation. These Modelica-compliant models are briefly described in the following paper of the 12th International Modelica Conference:

> Mohammed Ahsan Adib Murad and Luigi Vanfretti, "A Modelica VSC-HVDC Average Value Model Implementation and its Software-to-Software Validation using an EMT Power System Domain Specific Simulator," Proceedings of the 12th International Modelica Conference, May 15-17, 2017, Prague, Czech Republic. [http://dx.doi.org/10.3384/ecp17132241](https://modelica.org/events/modelica2017/proceedings/html/authors/author_286.html)

Please see the full paper for the documentation of the model.

The repository also contains the EMTP-RV simulation models to which the Modelica implementation was compared to.

## How to Simulate it?

Using Dymola, follow the steps below:
- File/Open `./VSCModelica/VSCHVDC.mo`
- Under the main package `VSCHVDC`, two subpackages should appear: (1) `HVDC_Emtp` and (2) `Example`
- Under the `Example` subpackage, go to the subpackage `VSC` and select the model `TwoNodePowerSystem`; the figure shown on top of this repository should be displayed.
- Go to the `Simulation` tab of your tool, and click the `Simulate` button.
- The simulaiton results should be similar to the ones shown next:
![alt text](https://github.com/ALSETLab/2017_ModelicaConf_VSC-HVDC_AVM_Model/blob/master/Example_Results/Dymola2018/VSC/TwoNodePowerSystem_currents.png)
![alt text](https://github.com/ALSETLab/2017_ModelicaConf_VSC-HVDC_AVM_Model/blob/master/Example_Results/Dymola2018/VSC/TwoNodePowerSystem_controls.png)
- You can perform similar simulations of some of the components within the model using the other models under the `Examples` subpackage, or by making your own simulation model.


## Running the models in OpenModelica
To run this model in OpenModelica, the following test was conducted:
- Download OpenModelica version 1.13.2 (64-bit).
- Go to `Tools>Options>Simulation`
- Add the following flag to the `OMC Command Line Options`: `-d=newInst`

While this allows you to simulate the model, it does not imply that the results from the simulation will be correct.
You will need to check carefuly the settings you are providing for simulation.

Currently, we have not found a configuration that would give acceptable results as compared to Dymola or EMTP-RV, as it can be observed in the figure below. To check your results, reference results from Dymola are provided under the `.\Example_Results\` directory. In the figure below, the red line corresponds to the references results, and the blue line corresponds to the simulation from OpenModelica.

![alt text](https://github.com/ALSETLab/2017_ModelicaConf_VSC-HVDC_AVM_Model/blob/master/Example_Results/openmodelica_v1p13p2_64bit.png)


## Development and contribution

The library is developed by [Mohammed Ahsan Adib Murad](https://github.com/ahsanKTH) and [Luigi Vanfretti](https://github.com/lvanfretti).

Contributions are welcome via pull requests.

## License - No Warranty

This Modelica package is free software and the use is completely at your own risk; it can be redistributed and/or modified under the terms of the GNU Public License version 3.

Copyright (C) 2017, Mohammed Ahsan Adidb Murad and Luigi Vanfretti
