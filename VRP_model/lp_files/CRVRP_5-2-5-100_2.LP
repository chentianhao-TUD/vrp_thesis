\ Model CRVRP
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  0 x[0,0] + 0 x[13,0] + 0 x[13,1] + 0 x[13,2] + 0 x[13,3] + 0 x[13,4]
   + 0 x[13,5] + 0 x[13,6] + 0 x[13,7] + 0 x[13,8] + 0 x[13,9] + 0 x[13,10]
   + 0 x[13,11] + 0 x[13,12] + 0 x[13,13] + 0 w[0,0] + 0 w[13,0]
   + 0 w[13,1] + 0 w[13,12] + 0 w[13,13] + 0 v[0,0] + 0 v[1,0] + 0 v[2,0]
   + 0 v[3,0] + 0 v[4,0] + 0 v[5,0] + 0 v[6,0] + 0 v[7,0] + 0 v[8,0]
   + 0 v[9,0] + 0 v[10,0] + 0 v[11,0] + 0 v[12,0] + 0 v[13,0] + 0 v[13,1]
   + 0 v[13,2] + 0 v[13,3] + 0 v[13,4] + 0 v[13,5] + 0 v[13,6] + 0 v[13,7]
   + 0 v[13,8] + 0 v[13,9] + 0 v[13,10] + 0 v[13,11] + 0 v[13,12]
   + 0 v[13,13] + 0 service_time[13] + start_service[13]
Subject To
 MaxNumberVehiclesStart: x[0,1] + x[0,2] + x[0,3] + x[0,4] + x[0,5]
   + x[0,6] + x[0,7] + x[0,8] + x[0,9] + x[0,10] + x[0,11] + x[0,12]
   + x[0,13] <= 2
 OutFlowVehicles[1]: x[1,1] + x[1,2] + x[1,3] + x[1,4] + x[1,5] + x[1,6]
   + x[1,7] + x[1,8] + x[1,9] + x[1,10] + x[1,11] + x[1,12] + x[1,13] = 1
 OutFlowVehicles[2]: x[2,1] + x[2,2] + x[2,3] + x[2,4] + x[2,5] + x[2,6]
   + x[2,7] + x[2,8] + x[2,9] + x[2,10] + x[2,11] + x[2,12] + x[2,13] = 1
 OutFlowVehicles[3]: x[3,1] + x[3,2] + x[3,3] + x[3,4] + x[3,5] + x[3,6]
   + x[3,7] + x[3,8] + x[3,9] + x[3,10] + x[3,11] + x[3,12] + x[3,13] = 1
 OutFlowVehicles[4]: x[4,1] + x[4,2] + x[4,3] + x[4,4] + x[4,5] + x[4,6]
   + x[4,7] + x[4,8] + x[4,9] + x[4,10] + x[4,11] + x[4,12] + x[4,13] = 1
 OutFlowVehicles[5]: x[5,1] + x[5,2] + x[5,3] + x[5,4] + x[5,5] + x[5,6]
   + x[5,7] + x[5,8] + x[5,9] + x[5,10] + x[5,11] + x[5,12] + x[5,13] = 1
 OutFlowVehicles[6]: x[6,1] + x[6,2] + x[6,3] + x[6,4] + x[6,5] + x[6,6]
   + x[6,7] + x[6,8] + x[6,9] + x[6,10] + x[6,11] + x[6,12] + x[6,13] = 1
 OutFlowVehicles[7]: x[7,1] + x[7,2] + x[7,3] + x[7,4] + x[7,5] + x[7,6]
   + x[7,7] + x[7,8] + x[7,9] + x[7,10] + x[7,11] + x[7,12] + x[7,13] = 1
 OutFlowVehicles[8]: x[8,1] + x[8,2] + x[8,3] + x[8,4] + x[8,5] + x[8,6]
   + x[8,7] + x[8,8] + x[8,9] + x[8,10] + x[8,11] + x[8,12] + x[8,13] = 1
 OutFlowVehicles[9]: x[9,1] + x[9,2] + x[9,3] + x[9,4] + x[9,5] + x[9,6]
   + x[9,7] + x[9,8] + x[9,9] + x[9,10] + x[9,11] + x[9,12] + x[9,13] = 1
 OutFlowVehicles[10]: x[10,1] + x[10,2] + x[10,3] + x[10,4] + x[10,5]
   + x[10,6] + x[10,7] + x[10,8] + x[10,9] + x[10,10] + x[10,11] + x[10,12]
   + x[10,13] = 1
 OutFlowVehicles[11]: x[11,1] + x[11,2] + x[11,3] + x[11,4] + x[11,5]
   + x[11,6] + x[11,7] + x[11,8] + x[11,9] + x[11,10] + x[11,11] + x[11,12]
   + x[11,13] = 1
 OutFlowVehicles[12]: x[12,1] + x[12,2] + x[12,3] + x[12,4] + x[12,5]
   + x[12,6] + x[12,7] + x[12,8] + x[12,9] + x[12,10] + x[12,11] + x[12,12]
   + x[12,13] = 1
 InFlowVehicles[1]: x[0,1] + x[1,1] + x[2,1] + x[3,1] + x[4,1] + x[5,1]
   + x[6,1] + x[7,1] + x[8,1] + x[9,1] + x[10,1] + x[11,1] + x[12,1] = 1
 InFlowVehicles[2]: x[0,2] + x[1,2] + x[2,2] + x[3,2] + x[4,2] + x[5,2]
   + x[6,2] + x[7,2] + x[8,2] + x[9,2] + x[10,2] + x[11,2] + x[12,2] = 1
 InFlowVehicles[3]: x[0,3] + x[1,3] + x[2,3] + x[3,3] + x[4,3] + x[5,3]
   + x[6,3] + x[7,3] + x[8,3] + x[9,3] + x[10,3] + x[11,3] + x[12,3] = 1
 InFlowVehicles[4]: x[0,4] + x[1,4] + x[2,4] + x[3,4] + x[4,4] + x[5,4]
   + x[6,4] + x[7,4] + x[8,4] + x[9,4] + x[10,4] + x[11,4] + x[12,4] = 1
 InFlowVehicles[5]: x[0,5] + x[1,5] + x[2,5] + x[3,5] + x[4,5] + x[5,5]
   + x[6,5] + x[7,5] + x[8,5] + x[9,5] + x[10,5] + x[11,5] + x[12,5] = 1
 InFlowVehicles[6]: x[0,6] + x[1,6] + x[2,6] + x[3,6] + x[4,6] + x[5,6]
   + x[6,6] + x[7,6] + x[8,6] + x[9,6] + x[10,6] + x[11,6] + x[12,6] = 1
 InFlowVehicles[7]: x[0,7] + x[1,7] + x[2,7] + x[3,7] + x[4,7] + x[5,7]
   + x[6,7] + x[7,7] + x[8,7] + x[9,7] + x[10,7] + x[11,7] + x[12,7] = 1
 InFlowVehicles[8]: x[0,8] + x[1,8] + x[2,8] + x[3,8] + x[4,8] + x[5,8]
   + x[6,8] + x[7,8] + x[8,8] + x[9,8] + x[10,8] + x[11,8] + x[12,8] = 1
 InFlowVehicles[9]: x[0,9] + x[1,9] + x[2,9] + x[3,9] + x[4,9] + x[5,9]
   + x[6,9] + x[7,9] + x[8,9] + x[9,9] + x[10,9] + x[11,9] + x[12,9] = 1
 InFlowVehicles[10]: x[0,10] + x[1,10] + x[2,10] + x[3,10] + x[4,10]
   + x[5,10] + x[6,10] + x[7,10] + x[8,10] + x[9,10] + x[10,10] + x[11,10]
   + x[12,10] = 1
 InFlowVehicles[11]: x[0,11] + x[1,11] + x[2,11] + x[3,11] + x[4,11]
   + x[5,11] + x[6,11] + x[7,11] + x[8,11] + x[9,11] + x[10,11] + x[11,11]
   + x[12,11] = 1
 InFlowVehicles[12]: x[0,12] + x[1,12] + x[2,12] + x[3,12] + x[4,12]
   + x[5,12] + x[6,12] + x[7,12] + x[8,12] + x[9,12] + x[10,12] + x[11,12]
   + x[12,12] = 1
 NodeArrivalVehicles[0,1]: - 999999 x[0,1] - start_service[0]
   + start_service[1] >= -999999
 NodeArrivalVehicles[0,2]: - 999999 x[0,2] - start_service[0]
   + start_service[2] >= -999954.5446932811
 NodeArrivalVehicles[0,3]: - 999999 x[0,3] - start_service[0]
   + start_service[3] >= -999934.6400107942
 NodeArrivalVehicles[0,4]: - 999999 x[0,4] - start_service[0]
   + start_service[4] >= -999968.7774252075
 NodeArrivalVehicles[0,5]: - 999999 x[0,5] - start_service[0]
   + start_service[5] >= -999946.3820795887
 NodeArrivalVehicles[0,6]: - 999999 x[0,6] - start_service[0]
   + start_service[6] >= -999945.8158556612
 NodeArrivalVehicles[0,7]: - 999999 x[0,7] - start_service[0]
   + start_service[7] >= -999954.5446932811
 NodeArrivalVehicles[0,8]: - 999999 x[0,8] - start_service[0]
   + start_service[8] >= -999934.6400107942
 NodeArrivalVehicles[0,9]: - 999999 x[0,9] - start_service[0]
   + start_service[9] >= -999968.7774252075
 NodeArrivalVehicles[0,10]: - 999999 x[0,10] - start_service[0]
   + start_service[10] >= -999946.3820795887
 NodeArrivalVehicles[0,11]: - 999999 x[0,11] - start_service[0]
   + start_service[11] >= -999945.8158556612
 NodeArrivalVehicles[0,12]: - 999999 x[0,12] - start_service[0]
   + start_service[12] >= -999999
 NodeArrivalVehicles[0,13]: - 999999 x[0,13] - start_service[0]
   + start_service[13] >= -999999
 NodeArrivalVehicles[1,1]: - 999999 x[1,1] >= -999999
 NodeArrivalVehicles[1,2]: - 999999 x[1,2] - start_service[1]
   + start_service[2] >= -999954.5446932811
 NodeArrivalVehicles[1,3]: - 999999 x[1,3] - start_service[1]
   + start_service[3] >= -999934.6400107942
 NodeArrivalVehicles[1,4]: - 999999 x[1,4] - start_service[1]
   + start_service[4] >= -999968.7774252075
 NodeArrivalVehicles[1,5]: - 999999 x[1,5] - start_service[1]
   + start_service[5] >= -999946.3820795887
 NodeArrivalVehicles[1,6]: - 999999 x[1,6] - start_service[1]
   + start_service[6] >= -999945.8158556612
 NodeArrivalVehicles[1,7]: - 999999 x[1,7] - start_service[1]
   + start_service[7] >= -999954.5446932811
 NodeArrivalVehicles[1,8]: - 999999 x[1,8] - start_service[1]
   + start_service[8] >= -999934.6400107942
 NodeArrivalVehicles[1,9]: - 999999 x[1,9] - start_service[1]
   + start_service[9] >= -999968.7774252075
 NodeArrivalVehicles[1,10]: - 999999 x[1,10] - start_service[1]
   + start_service[10] >= -999946.3820795887
 NodeArrivalVehicles[1,11]: - 999999 x[1,11] - start_service[1]
   + start_service[11] >= -999945.8158556612
 NodeArrivalVehicles[1,12]: - 999999 x[1,12] - start_service[1]
   + start_service[12] >= -999999
 NodeArrivalVehicles[1,13]: - 999999 x[1,13] - start_service[1]
   + start_service[13] >= -999999
 NodeArrivalVehicles[2,1]: - 999999 x[2,1] + start_service[1]
   - start_service[2] >= -999954.5446932811
 NodeArrivalVehicles[2,2]: - 999999 x[2,2] >= -999999
 NodeArrivalVehicles[2,3]: - 999999 x[2,3] - start_service[2]
   + start_service[3] >= -999900.8745887573
 NodeArrivalVehicles[2,4]: - 999999 x[2,4] - start_service[2]
   + start_service[4] >= -999943.5901960946
 NodeArrivalVehicles[2,5]: - 999999 x[2,5] - start_service[2]
   + start_service[5] >= -999904.5210086526
 NodeArrivalVehicles[2,6]: - 999999 x[2,6] - start_service[2]
   + start_service[6] >= -999912.5492633492
 NodeArrivalVehicles[2,7]: - 999999 x[2,7] - start_service[2]
   + start_service[7] >= -999999
 NodeArrivalVehicles[2,8]: - 999999 x[2,8] - start_service[2]
   + start_service[8] >= -999900.8745887573
 NodeArrivalVehicles[2,9]: - 999999 x[2,9] - start_service[2]
   + start_service[9] >= -999943.5901960946
 NodeArrivalVehicles[2,10]: - 999999 x[2,10] - start_service[2]
   + start_service[10] >= -999904.5210086526
 NodeArrivalVehicles[2,11]: - 999999 x[2,11] - start_service[2]
   + start_service[11] >= -999912.5492633492
 NodeArrivalVehicles[2,12]: - 999999 x[2,12] - start_service[2]
   + start_service[12] >= -999954.5446932811
 NodeArrivalVehicles[2,13]: - 999999 x[2,13] - start_service[2]
   + start_service[13] >= -999954.5446932811
 NodeArrivalVehicles[3,1]: - 999999 x[3,1] + start_service[1]
   - start_service[3] >= -999934.6400107942
 NodeArrivalVehicles[3,2]: - 999999 x[3,2] + start_service[2]
   - start_service[3] >= -999900.8745887573
 NodeArrivalVehicles[3,3]: - 999999 x[3,3] >= -999999
 NodeArrivalVehicles[3,4]: - 999999 x[3,4] - start_service[3]
   + start_service[4] >= -999956.223568582
 NodeArrivalVehicles[3,5]: - 999999 x[3,5] - start_service[3]
   + start_service[5] >= -999924.1893788762
 NodeArrivalVehicles[3,6]: - 999999 x[3,6] - start_service[3]
   + start_service[6] >= -999903.7963930037
 NodeArrivalVehicles[3,7]: - 999999 x[3,7] - start_service[3]
   + start_service[7] >= -999900.8745887573
 NodeArrivalVehicles[3,8]: - 999999 x[3,8] - start_service[3]
   + start_service[8] >= -999999
 NodeArrivalVehicles[3,9]: - 999999 x[3,9] - start_service[3]
   + start_service[9] >= -999956.223568582
 NodeArrivalVehicles[3,10]: - 999999 x[3,10] - start_service[3]
   + start_service[10] >= -999924.1893788762
 NodeArrivalVehicles[3,11]: - 999999 x[3,11] - start_service[3]
   + start_service[11] >= -999903.7963930037
 NodeArrivalVehicles[3,12]: - 999999 x[3,12] - start_service[3]
   + start_service[12] >= -999934.6400107942
 NodeArrivalVehicles[3,13]: - 999999 x[3,13] - start_service[3]
   + start_service[13] >= -999934.6400107942
 NodeArrivalVehicles[4,1]: - 999999 x[4,1] + start_service[1]
   - start_service[4] >= -999968.7774252075
 NodeArrivalVehicles[4,2]: - 999999 x[4,2] + start_service[2]
   - start_service[4] >= -999943.5901960946
 NodeArrivalVehicles[4,3]: - 999999 x[4,3] + start_service[3]
   - start_service[4] >= -999956.223568582
 NodeArrivalVehicles[4,4]: - 999999 x[4,4] >= -999999
 NodeArrivalVehicles[4,5]: - 999999 x[4,5] - start_service[4]
   + start_service[5] >= -999928.8980723232
 NodeArrivalVehicles[4,6]: - 999999 x[4,6] - start_service[4]
   + start_service[6] >= -999919.8307914401
 NodeArrivalVehicles[4,7]: - 999999 x[4,7] - start_service[4]
   + start_service[7] >= -999943.5901960946
 NodeArrivalVehicles[4,8]: - 999999 x[4,8] - start_service[4]
   + start_service[8] >= -999956.223568582
 NodeArrivalVehicles[4,9]: - 999999 x[4,9] - start_service[4]
   + start_service[9] >= -999999
 NodeArrivalVehicles[4,10]: - 999999 x[4,10] - start_service[4]
   + start_service[10] >= -999928.8980723232
 NodeArrivalVehicles[4,11]: - 999999 x[4,11] - start_service[4]
   + start_service[11] >= -999919.8307914401
 NodeArrivalVehicles[4,12]: - 999999 x[4,12] - start_service[4]
   + start_service[12] >= -999968.7774252075
 NodeArrivalVehicles[4,13]: - 999999 x[4,13] - start_service[4]
   + start_service[13] >= -999968.7774252075
 NodeArrivalVehicles[5,1]: - 999999 x[5,1] + start_service[1]
   - start_service[5] >= -999946.3820795887
 NodeArrivalVehicles[5,2]: - 999999 x[5,2] + start_service[2]
   - start_service[5] >= -999904.5210086526
 NodeArrivalVehicles[5,3]: - 999999 x[5,3] + start_service[3]
   - start_service[5] >= -999924.1893788762
 NodeArrivalVehicles[5,4]: - 999999 x[5,4] + start_service[4]
   - start_service[5] >= -999928.8980723232
 NodeArrivalVehicles[5,5]: - 999999 x[5,5] >= -999999
 NodeArrivalVehicles[5,6]: - 999999 x[5,6] - start_service[5]
   + start_service[6] >= -999972.5120726503
 NodeArrivalVehicles[5,7]: - 999999 x[5,7] - start_service[5]
   + start_service[7] >= -999904.5210086526
 NodeArrivalVehicles[5,8]: - 999999 x[5,8] - start_service[5]
   + start_service[8] >= -999924.1893788762
 NodeArrivalVehicles[5,9]: - 999999 x[5,9] - start_service[5]
   + start_service[9] >= -999928.8980723232
 NodeArrivalVehicles[5,10]: - 999999 x[5,10] - start_service[5]
   + start_service[10] >= -999999
 NodeArrivalVehicles[5,11]: - 999999 x[5,11] - start_service[5]
   + start_service[11] >= -999972.5120726503
 NodeArrivalVehicles[5,12]: - 999999 x[5,12] - start_service[5]
   + start_service[12] >= -999946.3820795887
 NodeArrivalVehicles[5,13]: - 999999 x[5,13] - start_service[5]
   + start_service[13] >= -999946.3820795887
 NodeArrivalVehicles[6,1]: - 999999 x[6,1] + start_service[1]
   - start_service[6] >= -999945.8158556612
 NodeArrivalVehicles[6,2]: - 999999 x[6,2] + start_service[2]
   - start_service[6] >= -999912.5492633492
 NodeArrivalVehicles[6,3]: - 999999 x[6,3] + start_service[3]
   - start_service[6] >= -999903.7963930037
 NodeArrivalVehicles[6,4]: - 999999 x[6,4] + start_service[4]
   - start_service[6] >= -999919.8307914401
 NodeArrivalVehicles[6,5]: - 999999 x[6,5] + start_service[5]
   - start_service[6] >= -999972.5120726503
 NodeArrivalVehicles[6,6]: - 999999 x[6,6] >= -999999
 NodeArrivalVehicles[6,7]: - 999999 x[6,7] - start_service[6]
   + start_service[7] >= -999912.5492633492
 NodeArrivalVehicles[6,8]: - 999999 x[6,8] - start_service[6]
   + start_service[8] >= -999903.7963930037
 NodeArrivalVehicles[6,9]: - 999999 x[6,9] - start_service[6]
   + start_service[9] >= -999919.8307914401
 NodeArrivalVehicles[6,10]: - 999999 x[6,10] - start_service[6]
   + start_service[10] >= -999972.5120726503
 NodeArrivalVehicles[6,11]: - 999999 x[6,11] - start_service[6]
   + start_service[11] >= -999999
 NodeArrivalVehicles[6,12]: - 999999 x[6,12] - start_service[6]
   + start_service[12] >= -999945.8158556612
 NodeArrivalVehicles[6,13]: - 999999 x[6,13] - start_service[6]
   + start_service[13] >= -999945.8158556612
 NodeArrivalVehicles[7,1]: - 999999 x[7,1] + start_service[1]
   - start_service[7] >= -999954.5446932811
 NodeArrivalVehicles[7,2]: - 999999 x[7,2] + start_service[2]
   - start_service[7] >= -999999
 NodeArrivalVehicles[7,3]: - 999999 x[7,3] + start_service[3]
   - start_service[7] >= -999900.8745887573
 NodeArrivalVehicles[7,4]: - 999999 x[7,4] + start_service[4]
   - start_service[7] >= -999943.5901960946
 NodeArrivalVehicles[7,5]: - 999999 x[7,5] + start_service[5]
   - start_service[7] >= -999904.5210086526
 NodeArrivalVehicles[7,6]: - 999999 x[7,6] + start_service[6]
   - start_service[7] >= -999912.5492633492
 NodeArrivalVehicles[7,7]: - 999999 x[7,7] >= -999999
 NodeArrivalVehicles[7,8]: - 999999 x[7,8] - start_service[7]
   + start_service[8] >= -999900.8745887573
 NodeArrivalVehicles[7,9]: - 999999 x[7,9] - start_service[7]
   + start_service[9] >= -999943.5901960946
 NodeArrivalVehicles[7,10]: - 999999 x[7,10] - start_service[7]
   + start_service[10] >= -999904.5210086526
 NodeArrivalVehicles[7,11]: - 999999 x[7,11] - start_service[7]
   + start_service[11] >= -999912.5492633492
 NodeArrivalVehicles[7,12]: - 999999 x[7,12] - start_service[7]
   + start_service[12] >= -999954.5446932811
 NodeArrivalVehicles[7,13]: - 999999 x[7,13] - start_service[7]
   + start_service[13] >= -999954.5446932811
 NodeArrivalVehicles[8,1]: - 999999 x[8,1] + start_service[1]
   - start_service[8] >= -999934.6400107942
 NodeArrivalVehicles[8,2]: - 999999 x[8,2] + start_service[2]
   - start_service[8] >= -999900.8745887573
 NodeArrivalVehicles[8,3]: - 999999 x[8,3] + start_service[3]
   - start_service[8] >= -999999
 NodeArrivalVehicles[8,4]: - 999999 x[8,4] + start_service[4]
   - start_service[8] >= -999956.223568582
 NodeArrivalVehicles[8,5]: - 999999 x[8,5] + start_service[5]
   - start_service[8] >= -999924.1893788762
 NodeArrivalVehicles[8,6]: - 999999 x[8,6] + start_service[6]
   - start_service[8] >= -999903.7963930037
 NodeArrivalVehicles[8,7]: - 999999 x[8,7] + start_service[7]
   - start_service[8] >= -999900.8745887573
 NodeArrivalVehicles[8,8]: - 999999 x[8,8] >= -999999
 NodeArrivalVehicles[8,9]: - 999999 x[8,9] - start_service[8]
   + start_service[9] >= -999956.223568582
 NodeArrivalVehicles[8,10]: - 999999 x[8,10] - start_service[8]
   + start_service[10] >= -999924.1893788762
 NodeArrivalVehicles[8,11]: - 999999 x[8,11] - start_service[8]
   + start_service[11] >= -999903.7963930037
 NodeArrivalVehicles[8,12]: - 999999 x[8,12] - start_service[8]
   + start_service[12] >= -999934.6400107942
 NodeArrivalVehicles[8,13]: - 999999 x[8,13] - start_service[8]
   + start_service[13] >= -999934.6400107942
 NodeArrivalVehicles[9,1]: - 999999 x[9,1] + start_service[1]
   - start_service[9] >= -999968.7774252075
 NodeArrivalVehicles[9,2]: - 999999 x[9,2] + start_service[2]
   - start_service[9] >= -999943.5901960946
 NodeArrivalVehicles[9,3]: - 999999 x[9,3] + start_service[3]
   - start_service[9] >= -999956.223568582
 NodeArrivalVehicles[9,4]: - 999999 x[9,4] + start_service[4]
   - start_service[9] >= -999999
 NodeArrivalVehicles[9,5]: - 999999 x[9,5] + start_service[5]
   - start_service[9] >= -999928.8980723232
 NodeArrivalVehicles[9,6]: - 999999 x[9,6] + start_service[6]
   - start_service[9] >= -999919.8307914401
 NodeArrivalVehicles[9,7]: - 999999 x[9,7] + start_service[7]
   - start_service[9] >= -999943.5901960946
 NodeArrivalVehicles[9,8]: - 999999 x[9,8] + start_service[8]
   - start_service[9] >= -999956.223568582
 NodeArrivalVehicles[9,9]: - 999999 x[9,9] >= -999999
 NodeArrivalVehicles[9,10]: - 999999 x[9,10] - start_service[9]
   + start_service[10] >= -999928.8980723232
 NodeArrivalVehicles[9,11]: - 999999 x[9,11] - start_service[9]
   + start_service[11] >= -999919.8307914401
 NodeArrivalVehicles[9,12]: - 999999 x[9,12] - start_service[9]
   + start_service[12] >= -999968.7774252075
 NodeArrivalVehicles[9,13]: - 999999 x[9,13] - start_service[9]
   + start_service[13] >= -999968.7774252075
 NodeArrivalVehicles[10,1]: - 999999 x[10,1] + start_service[1]
   - start_service[10] >= -999946.3820795887
 NodeArrivalVehicles[10,2]: - 999999 x[10,2] + start_service[2]
   - start_service[10] >= -999904.5210086526
 NodeArrivalVehicles[10,3]: - 999999 x[10,3] + start_service[3]
   - start_service[10] >= -999924.1893788762
 NodeArrivalVehicles[10,4]: - 999999 x[10,4] + start_service[4]
   - start_service[10] >= -999928.8980723232
 NodeArrivalVehicles[10,5]: - 999999 x[10,5] + start_service[5]
   - start_service[10] >= -999999
 NodeArrivalVehicles[10,6]: - 999999 x[10,6] + start_service[6]
   - start_service[10] >= -999972.5120726503
 NodeArrivalVehicles[10,7]: - 999999 x[10,7] + start_service[7]
   - start_service[10] >= -999904.5210086526
 NodeArrivalVehicles[10,8]: - 999999 x[10,8] + start_service[8]
   - start_service[10] >= -999924.1893788762
 NodeArrivalVehicles[10,9]: - 999999 x[10,9] + start_service[9]
   - start_service[10] >= -999928.8980723232
 NodeArrivalVehicles[10,10]: - 999999 x[10,10] >= -999999
 NodeArrivalVehicles[10,11]: - 999999 x[10,11] - start_service[10]
   + start_service[11] >= -999972.5120726503
 NodeArrivalVehicles[10,12]: - 999999 x[10,12] - start_service[10]
   + start_service[12] >= -999946.3820795887
 NodeArrivalVehicles[10,13]: - 999999 x[10,13] - start_service[10]
   + start_service[13] >= -999946.3820795887
 NodeArrivalVehicles[11,1]: - 999999 x[11,1] + start_service[1]
   - start_service[11] >= -999945.8158556612
 NodeArrivalVehicles[11,2]: - 999999 x[11,2] + start_service[2]
   - start_service[11] >= -999912.5492633492
 NodeArrivalVehicles[11,3]: - 999999 x[11,3] + start_service[3]
   - start_service[11] >= -999903.7963930037
 NodeArrivalVehicles[11,4]: - 999999 x[11,4] + start_service[4]
   - start_service[11] >= -999919.8307914401
 NodeArrivalVehicles[11,5]: - 999999 x[11,5] + start_service[5]
   - start_service[11] >= -999972.5120726503
 NodeArrivalVehicles[11,6]: - 999999 x[11,6] + start_service[6]
   - start_service[11] >= -999999
 NodeArrivalVehicles[11,7]: - 999999 x[11,7] + start_service[7]
   - start_service[11] >= -999912.5492633492
 NodeArrivalVehicles[11,8]: - 999999 x[11,8] + start_service[8]
   - start_service[11] >= -999903.7963930037
 NodeArrivalVehicles[11,9]: - 999999 x[11,9] + start_service[9]
   - start_service[11] >= -999919.8307914401
 NodeArrivalVehicles[11,10]: - 999999 x[11,10] + start_service[10]
   - start_service[11] >= -999972.5120726503
 NodeArrivalVehicles[11,11]: - 999999 x[11,11] >= -999999
 NodeArrivalVehicles[11,12]: - 999999 x[11,12] - start_service[11]
   + start_service[12] >= -999945.8158556612
 NodeArrivalVehicles[11,13]: - 999999 x[11,13] - start_service[11]
   + start_service[13] >= -999945.8158556612
 NodeArrivalVehicles[12,1]: - 999999 x[12,1] + start_service[1]
   - start_service[12] >= -999999
 NodeArrivalVehicles[12,2]: - 999999 x[12,2] + start_service[2]
   - start_service[12] >= -999954.5446932811
 NodeArrivalVehicles[12,3]: - 999999 x[12,3] + start_service[3]
   - start_service[12] >= -999934.6400107942
 NodeArrivalVehicles[12,4]: - 999999 x[12,4] + start_service[4]
   - start_service[12] >= -999968.7774252075
 NodeArrivalVehicles[12,5]: - 999999 x[12,5] + start_service[5]
   - start_service[12] >= -999946.3820795887
 NodeArrivalVehicles[12,6]: - 999999 x[12,6] + start_service[6]
   - start_service[12] >= -999945.8158556612
 NodeArrivalVehicles[12,7]: - 999999 x[12,7] + start_service[7]
   - start_service[12] >= -999954.5446932811
 NodeArrivalVehicles[12,8]: - 999999 x[12,8] + start_service[8]
   - start_service[12] >= -999934.6400107942
 NodeArrivalVehicles[12,9]: - 999999 x[12,9] + start_service[9]
   - start_service[12] >= -999968.7774252075
 NodeArrivalVehicles[12,10]: - 999999 x[12,10] + start_service[10]
   - start_service[12] >= -999946.3820795887
 NodeArrivalVehicles[12,11]: - 999999 x[12,11] + start_service[11]
   - start_service[12] >= -999945.8158556612
 NodeArrivalVehicles[12,12]: - 999999 x[12,12] >= -999999
 NodeArrivalVehicles[12,13]: - 999999 x[12,13] - start_service[12]
   + start_service[13] >= -999999
 RobotsStartAtVehicleDepot: w[0,1] + w[0,2] + w[0,3] + w[0,4] + w[0,5]
   + w[0,6] + w[0,7] + w[0,8] + w[0,9] + w[0,10] + w[0,11] + w[0,12]
   + w[0,13] <= 4
 TotalRobotStarts: w[0,2] + w[0,3] + w[0,4] + w[0,5] + w[0,6] + w[0,7]
   + w[0,8] + w[0,9] + w[0,10] + w[0,11] + w[0,12] + w[0,13] + w[1,2]
   + w[1,3] + w[1,4] + w[1,5] + w[1,6] + w[1,7] + w[1,8] + w[1,9] + w[1,10]
   + w[1,11] + w[1,12] + w[1,13] <= 5
 RobotsStartAtRobotDepot: w[1,1] + w[1,2] + w[1,3] + w[1,4] + w[1,5]
   + w[1,6] + w[1,7] + w[1,8] + w[1,9] + w[1,10] + w[1,11] + w[1,12]
   + w[1,13] <= 1
 RobotsFinishAtVehicleDepot: w[0,13] + w[1,13] + w[2,13] + w[3,13]
   + w[4,13] + w[5,13] + w[6,13] + w[7,13] + w[8,13] + w[9,13] + w[10,13]
   + w[11,13] + w[12,13] <= 4
 TotalRobotFinishes: w[0,12] + w[0,13] + w[1,12] + w[1,13] + w[2,12]
   + w[2,13] + w[3,12] + w[3,13] + w[4,12] + w[4,13] + w[5,12] + w[5,13]
   + w[6,12] + w[6,13] + w[7,12] + w[7,13] + w[8,12] + w[8,13] + w[9,12]
   + w[9,13] + w[10,12] + w[10,13] + w[11,12] + w[11,13] <= 5
 FlowConservationRobots[2]: w[0,2] + w[1,2] - w[2,1] - w[2,3] - w[2,4]
   - w[2,5] - w[2,6] - w[2,7] - w[2,8] - w[2,9] - w[2,10] - w[2,11]
   - w[2,12] - w[2,13] + w[3,2] + w[4,2] + w[5,2] + w[6,2] + w[7,2]
   + w[8,2] + w[9,2] + w[10,2] + w[11,2] + w[12,2] = 0
 FlowConservationRobots[3]: w[0,3] + w[1,3] + w[2,3] - w[3,1] - w[3,2]
   - w[3,4] - w[3,5] - w[3,6] - w[3,7] - w[3,8] - w[3,9] - w[3,10]
   - w[3,11] - w[3,12] - w[3,13] + w[4,3] + w[5,3] + w[6,3] + w[7,3]
   + w[8,3] + w[9,3] + w[10,3] + w[11,3] + w[12,3] = 0
 FlowConservationRobots[4]: w[0,4] + w[1,4] + w[2,4] + w[3,4] - w[4,1]
   - w[4,2] - w[4,3] - w[4,5] - w[4,6] - w[4,7] - w[4,8] - w[4,9] - w[4,10]
   - w[4,11] - w[4,12] - w[4,13] + w[5,4] + w[6,4] + w[7,4] + w[8,4]
   + w[9,4] + w[10,4] + w[11,4] + w[12,4] = 0
 FlowConservationRobots[5]: w[0,5] + w[1,5] + w[2,5] + w[3,5] + w[4,5]
   - w[5,1] - w[5,2] - w[5,3] - w[5,4] - w[5,6] - w[5,7] - w[5,8] - w[5,9]
   - w[5,10] - w[5,11] - w[5,12] - w[5,13] + w[6,5] + w[7,5] + w[8,5]
   + w[9,5] + w[10,5] + w[11,5] + w[12,5] = 0
 FlowConservationRobots[6]: w[0,6] + w[1,6] + w[2,6] + w[3,6] + w[4,6]
   + w[5,6] - w[6,1] - w[6,2] - w[6,3] - w[6,4] - w[6,5] - w[6,7] - w[6,8]
   - w[6,9] - w[6,10] - w[6,11] - w[6,12] - w[6,13] + w[7,6] + w[8,6]
   + w[9,6] + w[10,6] + w[11,6] + w[12,6] = 0
 FlowConservationRobots[7]: w[0,7] + w[1,7] + w[2,7] + w[3,7] + w[4,7]
   + w[5,7] + w[6,7] - w[7,1] - w[7,2] - w[7,3] - w[7,4] - w[7,5] - w[7,6]
   - w[7,8] - w[7,9] - w[7,10] - w[7,11] - w[7,12] - w[7,13] + w[8,7]
   + w[9,7] + w[10,7] + w[11,7] + w[12,7] = 0
 FlowConservationRobots[8]: w[0,8] + w[1,8] + w[2,8] + w[3,8] + w[4,8]
   + w[5,8] + w[6,8] + w[7,8] - w[8,1] - w[8,2] - w[8,3] - w[8,4] - w[8,5]
   - w[8,6] - w[8,7] - w[8,9] - w[8,10] - w[8,11] - w[8,12] - w[8,13]
   + w[9,8] + w[10,8] + w[11,8] + w[12,8] = 0
 FlowConservationRobots[9]: w[0,9] + w[1,9] + w[2,9] + w[3,9] + w[4,9]
   + w[5,9] + w[6,9] + w[7,9] + w[8,9] - w[9,1] - w[9,2] - w[9,3] - w[9,4]
   - w[9,5] - w[9,6] - w[9,7] - w[9,8] - w[9,10] - w[9,11] - w[9,12]
   - w[9,13] + w[10,9] + w[11,9] + w[12,9] = 0
 FlowConservationRobots[10]: w[0,10] + w[1,10] + w[2,10] + w[3,10]
   + w[4,10] + w[5,10] + w[6,10] + w[7,10] + w[8,10] + w[9,10] - w[10,1]
   - w[10,2] - w[10,3] - w[10,4] - w[10,5] - w[10,6] - w[10,7] - w[10,8]
   - w[10,9] - w[10,11] - w[10,12] - w[10,13] + w[11,10] + w[12,10] = 0
 FlowConservationRobots[11]: w[0,11] + w[1,11] + w[2,11] + w[3,11]
   + w[4,11] + w[5,11] + w[6,11] + w[7,11] + w[8,11] + w[9,11] + w[10,11]
   - w[11,1] - w[11,2] - w[11,3] - w[11,4] - w[11,5] - w[11,6] - w[11,7]
   - w[11,8] - w[11,9] - w[11,10] - w[11,12] - w[11,13] + w[12,11] = 0
 MaxNumberSupportOnArc[0,1]: w[0,1] - 2 v[0,1] <= 0
 MaxNumberSupportOnArc[0,2]: w[0,2] - 2 v[0,2] <= 0
 MaxNumberSupportOnArc[0,3]: w[0,3] - 2 v[0,3] <= 0
 MaxNumberSupportOnArc[0,4]: w[0,4] - 2 v[0,4] <= 0
 MaxNumberSupportOnArc[0,5]: w[0,5] - 2 v[0,5] <= 0
 MaxNumberSupportOnArc[0,6]: w[0,6] - 2 v[0,6] <= 0
 MaxNumberSupportOnArc[0,7]: w[0,7] - 2 v[0,7] <= 0
 MaxNumberSupportOnArc[0,8]: w[0,8] - 2 v[0,8] <= 0
 MaxNumberSupportOnArc[0,9]: w[0,9] - 2 v[0,9] <= 0
 MaxNumberSupportOnArc[0,10]: w[0,10] - 2 v[0,10] <= 0
 MaxNumberSupportOnArc[0,11]: w[0,11] - 2 v[0,11] <= 0
 MaxNumberSupportOnArc[0,12]: w[0,12] - 2 v[0,12] <= 0
 MaxNumberSupportOnArc[0,13]: w[0,13] - 2 v[0,13] <= 0
 MaxNumberSupportOnArc[1,1]: w[1,1] - 2 v[1,1] <= 0
 MaxNumberSupportOnArc[1,2]: w[1,2] - 2 v[1,2] <= 0
 MaxNumberSupportOnArc[1,3]: w[1,3] - 2 v[1,3] <= 0
 MaxNumberSupportOnArc[1,4]: w[1,4] - 2 v[1,4] <= 0
 MaxNumberSupportOnArc[1,5]: w[1,5] - 2 v[1,5] <= 0
 MaxNumberSupportOnArc[1,6]: w[1,6] - 2 v[1,6] <= 0
 MaxNumberSupportOnArc[1,7]: w[1,7] - 2 v[1,7] <= 0
 MaxNumberSupportOnArc[1,8]: w[1,8] - 2 v[1,8] <= 0
 MaxNumberSupportOnArc[1,9]: w[1,9] - 2 v[1,9] <= 0
 MaxNumberSupportOnArc[1,10]: w[1,10] - 2 v[1,10] <= 0
 MaxNumberSupportOnArc[1,11]: w[1,11] - 2 v[1,11] <= 0
 MaxNumberSupportOnArc[1,12]: w[1,12] - 2 v[1,12] <= 0
 MaxNumberSupportOnArc[1,13]: w[1,13] - 2 v[1,13] <= 0
 MaxNumberSupportOnArc[2,1]: w[2,1] - 2 v[2,1] <= 0
 MaxNumberSupportOnArc[2,2]: w[2,2] - 2 v[2,2] <= 0
 MaxNumberSupportOnArc[2,3]: w[2,3] - 2 v[2,3] <= 0
 MaxNumberSupportOnArc[2,4]: w[2,4] - 2 v[2,4] <= 0
 MaxNumberSupportOnArc[2,5]: w[2,5] - 2 v[2,5] <= 0
 MaxNumberSupportOnArc[2,6]: w[2,6] - 2 v[2,6] <= 0
 MaxNumberSupportOnArc[2,7]: w[2,7] - 2 v[2,7] <= 0
 MaxNumberSupportOnArc[2,8]: w[2,8] - 2 v[2,8] <= 0
 MaxNumberSupportOnArc[2,9]: w[2,9] - 2 v[2,9] <= 0
 MaxNumberSupportOnArc[2,10]: w[2,10] - 2 v[2,10] <= 0
 MaxNumberSupportOnArc[2,11]: w[2,11] - 2 v[2,11] <= 0
 MaxNumberSupportOnArc[2,12]: w[2,12] - 2 v[2,12] <= 0
 MaxNumberSupportOnArc[2,13]: w[2,13] - 2 v[2,13] <= 0
 MaxNumberSupportOnArc[3,1]: w[3,1] - 2 v[3,1] <= 0
 MaxNumberSupportOnArc[3,2]: w[3,2] - 2 v[3,2] <= 0
 MaxNumberSupportOnArc[3,3]: w[3,3] - 2 v[3,3] <= 0
 MaxNumberSupportOnArc[3,4]: w[3,4] - 2 v[3,4] <= 0
 MaxNumberSupportOnArc[3,5]: w[3,5] - 2 v[3,5] <= 0
 MaxNumberSupportOnArc[3,6]: w[3,6] - 2 v[3,6] <= 0
 MaxNumberSupportOnArc[3,7]: w[3,7] - 2 v[3,7] <= 0
 MaxNumberSupportOnArc[3,8]: w[3,8] - 2 v[3,8] <= 0
 MaxNumberSupportOnArc[3,9]: w[3,9] - 2 v[3,9] <= 0
 MaxNumberSupportOnArc[3,10]: w[3,10] - 2 v[3,10] <= 0
 MaxNumberSupportOnArc[3,11]: w[3,11] - 2 v[3,11] <= 0
 MaxNumberSupportOnArc[3,12]: w[3,12] - 2 v[3,12] <= 0
 MaxNumberSupportOnArc[3,13]: w[3,13] - 2 v[3,13] <= 0
 MaxNumberSupportOnArc[4,1]: w[4,1] - 2 v[4,1] <= 0
 MaxNumberSupportOnArc[4,2]: w[4,2] - 2 v[4,2] <= 0
 MaxNumberSupportOnArc[4,3]: w[4,3] - 2 v[4,3] <= 0
 MaxNumberSupportOnArc[4,4]: w[4,4] - 2 v[4,4] <= 0
 MaxNumberSupportOnArc[4,5]: w[4,5] - 2 v[4,5] <= 0
 MaxNumberSupportOnArc[4,6]: w[4,6] - 2 v[4,6] <= 0
 MaxNumberSupportOnArc[4,7]: w[4,7] - 2 v[4,7] <= 0
 MaxNumberSupportOnArc[4,8]: w[4,8] - 2 v[4,8] <= 0
 MaxNumberSupportOnArc[4,9]: w[4,9] - 2 v[4,9] <= 0
 MaxNumberSupportOnArc[4,10]: w[4,10] - 2 v[4,10] <= 0
 MaxNumberSupportOnArc[4,11]: w[4,11] - 2 v[4,11] <= 0
 MaxNumberSupportOnArc[4,12]: w[4,12] - 2 v[4,12] <= 0
 MaxNumberSupportOnArc[4,13]: w[4,13] - 2 v[4,13] <= 0
 MaxNumberSupportOnArc[5,1]: w[5,1] - 2 v[5,1] <= 0
 MaxNumberSupportOnArc[5,2]: w[5,2] - 2 v[5,2] <= 0
 MaxNumberSupportOnArc[5,3]: w[5,3] - 2 v[5,3] <= 0
 MaxNumberSupportOnArc[5,4]: w[5,4] - 2 v[5,4] <= 0
 MaxNumberSupportOnArc[5,5]: w[5,5] - 2 v[5,5] <= 0
 MaxNumberSupportOnArc[5,6]: w[5,6] - 2 v[5,6] <= 0
 MaxNumberSupportOnArc[5,7]: w[5,7] - 2 v[5,7] <= 0
 MaxNumberSupportOnArc[5,8]: w[5,8] - 2 v[5,8] <= 0
 MaxNumberSupportOnArc[5,9]: w[5,9] - 2 v[5,9] <= 0
 MaxNumberSupportOnArc[5,10]: w[5,10] - 2 v[5,10] <= 0
 MaxNumberSupportOnArc[5,11]: w[5,11] - 2 v[5,11] <= 0
 MaxNumberSupportOnArc[5,12]: w[5,12] - 2 v[5,12] <= 0
 MaxNumberSupportOnArc[5,13]: w[5,13] - 2 v[5,13] <= 0
 MaxNumberSupportOnArc[6,1]: w[6,1] - 2 v[6,1] <= 0
 MaxNumberSupportOnArc[6,2]: w[6,2] - 2 v[6,2] <= 0
 MaxNumberSupportOnArc[6,3]: w[6,3] - 2 v[6,3] <= 0
 MaxNumberSupportOnArc[6,4]: w[6,4] - 2 v[6,4] <= 0
 MaxNumberSupportOnArc[6,5]: w[6,5] - 2 v[6,5] <= 0
 MaxNumberSupportOnArc[6,6]: w[6,6] - 2 v[6,6] <= 0
 MaxNumberSupportOnArc[6,7]: w[6,7] - 2 v[6,7] <= 0
 MaxNumberSupportOnArc[6,8]: w[6,8] - 2 v[6,8] <= 0
 MaxNumberSupportOnArc[6,9]: w[6,9] - 2 v[6,9] <= 0
 MaxNumberSupportOnArc[6,10]: w[6,10] - 2 v[6,10] <= 0
 MaxNumberSupportOnArc[6,11]: w[6,11] - 2 v[6,11] <= 0
 MaxNumberSupportOnArc[6,12]: w[6,12] - 2 v[6,12] <= 0
 MaxNumberSupportOnArc[6,13]: w[6,13] - 2 v[6,13] <= 0
 MaxNumberSupportOnArc[7,1]: w[7,1] - 2 v[7,1] <= 0
 MaxNumberSupportOnArc[7,2]: w[7,2] - 2 v[7,2] <= 0
 MaxNumberSupportOnArc[7,3]: w[7,3] - 2 v[7,3] <= 0
 MaxNumberSupportOnArc[7,4]: w[7,4] - 2 v[7,4] <= 0
 MaxNumberSupportOnArc[7,5]: w[7,5] - 2 v[7,5] <= 0
 MaxNumberSupportOnArc[7,6]: w[7,6] - 2 v[7,6] <= 0
 MaxNumberSupportOnArc[7,7]: w[7,7] - 2 v[7,7] <= 0
 MaxNumberSupportOnArc[7,8]: w[7,8] - 2 v[7,8] <= 0
 MaxNumberSupportOnArc[7,9]: w[7,9] - 2 v[7,9] <= 0
 MaxNumberSupportOnArc[7,10]: w[7,10] - 2 v[7,10] <= 0
 MaxNumberSupportOnArc[7,11]: w[7,11] - 2 v[7,11] <= 0
 MaxNumberSupportOnArc[7,12]: w[7,12] - 2 v[7,12] <= 0
 MaxNumberSupportOnArc[7,13]: w[7,13] - 2 v[7,13] <= 0
 MaxNumberSupportOnArc[8,1]: w[8,1] - 2 v[8,1] <= 0
 MaxNumberSupportOnArc[8,2]: w[8,2] - 2 v[8,2] <= 0
 MaxNumberSupportOnArc[8,3]: w[8,3] - 2 v[8,3] <= 0
 MaxNumberSupportOnArc[8,4]: w[8,4] - 2 v[8,4] <= 0
 MaxNumberSupportOnArc[8,5]: w[8,5] - 2 v[8,5] <= 0
 MaxNumberSupportOnArc[8,6]: w[8,6] - 2 v[8,6] <= 0
 MaxNumberSupportOnArc[8,7]: w[8,7] - 2 v[8,7] <= 0
 MaxNumberSupportOnArc[8,8]: w[8,8] - 2 v[8,8] <= 0
 MaxNumberSupportOnArc[8,9]: w[8,9] - 2 v[8,9] <= 0
 MaxNumberSupportOnArc[8,10]: w[8,10] - 2 v[8,10] <= 0
 MaxNumberSupportOnArc[8,11]: w[8,11] - 2 v[8,11] <= 0
 MaxNumberSupportOnArc[8,12]: w[8,12] - 2 v[8,12] <= 0
 MaxNumberSupportOnArc[8,13]: w[8,13] - 2 v[8,13] <= 0
 MaxNumberSupportOnArc[9,1]: w[9,1] - 2 v[9,1] <= 0
 MaxNumberSupportOnArc[9,2]: w[9,2] - 2 v[9,2] <= 0
 MaxNumberSupportOnArc[9,3]: w[9,3] - 2 v[9,3] <= 0
 MaxNumberSupportOnArc[9,4]: w[9,4] - 2 v[9,4] <= 0
 MaxNumberSupportOnArc[9,5]: w[9,5] - 2 v[9,5] <= 0
 MaxNumberSupportOnArc[9,6]: w[9,6] - 2 v[9,6] <= 0
 MaxNumberSupportOnArc[9,7]: w[9,7] - 2 v[9,7] <= 0
 MaxNumberSupportOnArc[9,8]: w[9,8] - 2 v[9,8] <= 0
 MaxNumberSupportOnArc[9,9]: w[9,9] - 2 v[9,9] <= 0
 MaxNumberSupportOnArc[9,10]: w[9,10] - 2 v[9,10] <= 0
 MaxNumberSupportOnArc[9,11]: w[9,11] - 2 v[9,11] <= 0
 MaxNumberSupportOnArc[9,12]: w[9,12] - 2 v[9,12] <= 0
 MaxNumberSupportOnArc[9,13]: w[9,13] - 2 v[9,13] <= 0
 MaxNumberSupportOnArc[10,1]: w[10,1] - 2 v[10,1] <= 0
 MaxNumberSupportOnArc[10,2]: w[10,2] - 2 v[10,2] <= 0
 MaxNumberSupportOnArc[10,3]: w[10,3] - 2 v[10,3] <= 0
 MaxNumberSupportOnArc[10,4]: w[10,4] - 2 v[10,4] <= 0
 MaxNumberSupportOnArc[10,5]: w[10,5] - 2 v[10,5] <= 0
 MaxNumberSupportOnArc[10,6]: w[10,6] - 2 v[10,6] <= 0
 MaxNumberSupportOnArc[10,7]: w[10,7] - 2 v[10,7] <= 0
 MaxNumberSupportOnArc[10,8]: w[10,8] - 2 v[10,8] <= 0
 MaxNumberSupportOnArc[10,9]: w[10,9] - 2 v[10,9] <= 0
 MaxNumberSupportOnArc[10,10]: w[10,10] - 2 v[10,10] <= 0
 MaxNumberSupportOnArc[10,11]: w[10,11] - 2 v[10,11] <= 0
 MaxNumberSupportOnArc[10,12]: w[10,12] - 2 v[10,12] <= 0
 MaxNumberSupportOnArc[10,13]: w[10,13] - 2 v[10,13] <= 0
 MaxNumberSupportOnArc[11,1]: w[11,1] - 2 v[11,1] <= 0
 MaxNumberSupportOnArc[11,2]: w[11,2] - 2 v[11,2] <= 0
 MaxNumberSupportOnArc[11,3]: w[11,3] - 2 v[11,3] <= 0
 MaxNumberSupportOnArc[11,4]: w[11,4] - 2 v[11,4] <= 0
 MaxNumberSupportOnArc[11,5]: w[11,5] - 2 v[11,5] <= 0
 MaxNumberSupportOnArc[11,6]: w[11,6] - 2 v[11,6] <= 0
 MaxNumberSupportOnArc[11,7]: w[11,7] - 2 v[11,7] <= 0
 MaxNumberSupportOnArc[11,8]: w[11,8] - 2 v[11,8] <= 0
 MaxNumberSupportOnArc[11,9]: w[11,9] - 2 v[11,9] <= 0
 MaxNumberSupportOnArc[11,10]: w[11,10] - 2 v[11,10] <= 0
 MaxNumberSupportOnArc[11,11]: w[11,11] - 2 v[11,11] <= 0
 MaxNumberSupportOnArc[11,12]: w[11,12] - 2 v[11,12] <= 0
 MaxNumberSupportOnArc[11,13]: w[11,13] - 2 v[11,13] <= 0
 MaxNumberSupportOnArc[12,1]: w[12,1] - 2 v[12,1] <= 0
 MaxNumberSupportOnArc[12,2]: w[12,2] - 2 v[12,2] <= 0
 MaxNumberSupportOnArc[12,3]: w[12,3] - 2 v[12,3] <= 0
 MaxNumberSupportOnArc[12,4]: w[12,4] - 2 v[12,4] <= 0
 MaxNumberSupportOnArc[12,5]: w[12,5] - 2 v[12,5] <= 0
 MaxNumberSupportOnArc[12,6]: w[12,6] - 2 v[12,6] <= 0
 MaxNumberSupportOnArc[12,7]: w[12,7] - 2 v[12,7] <= 0
 MaxNumberSupportOnArc[12,8]: w[12,8] - 2 v[12,8] <= 0
 MaxNumberSupportOnArc[12,9]: w[12,9] - 2 v[12,9] <= 0
 MaxNumberSupportOnArc[12,10]: w[12,10] - 2 v[12,10] <= 0
 MaxNumberSupportOnArc[12,11]: w[12,11] - 2 v[12,11] <= 0
 MaxNumberSupportOnArc[12,12]: w[12,12] - 2 v[12,12] <= 0
 MaxNumberSupportOnArc[12,13]: w[12,13] - 2 v[12,13] <= 0
 ArcUsedByRobot[0,1]: w[0,1] - v[0,1] >= 0
 ArcUsedByRobot[0,2]: w[0,2] - v[0,2] >= 0
 ArcUsedByRobot[0,3]: w[0,3] - v[0,3] >= 0
 ArcUsedByRobot[0,4]: w[0,4] - v[0,4] >= 0
 ArcUsedByRobot[0,5]: w[0,5] - v[0,5] >= 0
 ArcUsedByRobot[0,6]: w[0,6] - v[0,6] >= 0
 ArcUsedByRobot[0,7]: w[0,7] - v[0,7] >= 0
 ArcUsedByRobot[0,8]: w[0,8] - v[0,8] >= 0
 ArcUsedByRobot[0,9]: w[0,9] - v[0,9] >= 0
 ArcUsedByRobot[0,10]: w[0,10] - v[0,10] >= 0
 ArcUsedByRobot[0,11]: w[0,11] - v[0,11] >= 0
 ArcUsedByRobot[0,12]: w[0,12] - v[0,12] >= 0
 ArcUsedByRobot[0,13]: w[0,13] - v[0,13] >= 0
 ArcUsedByRobot[1,1]: w[1,1] - v[1,1] >= 0
 ArcUsedByRobot[1,2]: w[1,2] - v[1,2] >= 0
 ArcUsedByRobot[1,3]: w[1,3] - v[1,3] >= 0
 ArcUsedByRobot[1,4]: w[1,4] - v[1,4] >= 0
 ArcUsedByRobot[1,5]: w[1,5] - v[1,5] >= 0
 ArcUsedByRobot[1,6]: w[1,6] - v[1,6] >= 0
 ArcUsedByRobot[1,7]: w[1,7] - v[1,7] >= 0
 ArcUsedByRobot[1,8]: w[1,8] - v[1,8] >= 0
 ArcUsedByRobot[1,9]: w[1,9] - v[1,9] >= 0
 ArcUsedByRobot[1,10]: w[1,10] - v[1,10] >= 0
 ArcUsedByRobot[1,11]: w[1,11] - v[1,11] >= 0
 ArcUsedByRobot[1,12]: w[1,12] - v[1,12] >= 0
 ArcUsedByRobot[1,13]: w[1,13] - v[1,13] >= 0
 ArcUsedByRobot[2,1]: w[2,1] - v[2,1] >= 0
 ArcUsedByRobot[2,2]: w[2,2] - v[2,2] >= 0
 ArcUsedByRobot[2,3]: w[2,3] - v[2,3] >= 0
 ArcUsedByRobot[2,4]: w[2,4] - v[2,4] >= 0
 ArcUsedByRobot[2,5]: w[2,5] - v[2,5] >= 0
 ArcUsedByRobot[2,6]: w[2,6] - v[2,6] >= 0
 ArcUsedByRobot[2,7]: w[2,7] - v[2,7] >= 0
 ArcUsedByRobot[2,8]: w[2,8] - v[2,8] >= 0
 ArcUsedByRobot[2,9]: w[2,9] - v[2,9] >= 0
 ArcUsedByRobot[2,10]: w[2,10] - v[2,10] >= 0
 ArcUsedByRobot[2,11]: w[2,11] - v[2,11] >= 0
 ArcUsedByRobot[2,12]: w[2,12] - v[2,12] >= 0
 ArcUsedByRobot[2,13]: w[2,13] - v[2,13] >= 0
 ArcUsedByRobot[3,1]: w[3,1] - v[3,1] >= 0
 ArcUsedByRobot[3,2]: w[3,2] - v[3,2] >= 0
 ArcUsedByRobot[3,3]: w[3,3] - v[3,3] >= 0
 ArcUsedByRobot[3,4]: w[3,4] - v[3,4] >= 0
 ArcUsedByRobot[3,5]: w[3,5] - v[3,5] >= 0
 ArcUsedByRobot[3,6]: w[3,6] - v[3,6] >= 0
 ArcUsedByRobot[3,7]: w[3,7] - v[3,7] >= 0
 ArcUsedByRobot[3,8]: w[3,8] - v[3,8] >= 0
 ArcUsedByRobot[3,9]: w[3,9] - v[3,9] >= 0
 ArcUsedByRobot[3,10]: w[3,10] - v[3,10] >= 0
 ArcUsedByRobot[3,11]: w[3,11] - v[3,11] >= 0
 ArcUsedByRobot[3,12]: w[3,12] - v[3,12] >= 0
 ArcUsedByRobot[3,13]: w[3,13] - v[3,13] >= 0
 ArcUsedByRobot[4,1]: w[4,1] - v[4,1] >= 0
 ArcUsedByRobot[4,2]: w[4,2] - v[4,2] >= 0
 ArcUsedByRobot[4,3]: w[4,3] - v[4,3] >= 0
 ArcUsedByRobot[4,4]: w[4,4] - v[4,4] >= 0
 ArcUsedByRobot[4,5]: w[4,5] - v[4,5] >= 0
 ArcUsedByRobot[4,6]: w[4,6] - v[4,6] >= 0
 ArcUsedByRobot[4,7]: w[4,7] - v[4,7] >= 0
 ArcUsedByRobot[4,8]: w[4,8] - v[4,8] >= 0
 ArcUsedByRobot[4,9]: w[4,9] - v[4,9] >= 0
 ArcUsedByRobot[4,10]: w[4,10] - v[4,10] >= 0
 ArcUsedByRobot[4,11]: w[4,11] - v[4,11] >= 0
 ArcUsedByRobot[4,12]: w[4,12] - v[4,12] >= 0
 ArcUsedByRobot[4,13]: w[4,13] - v[4,13] >= 0
 ArcUsedByRobot[5,1]: w[5,1] - v[5,1] >= 0
 ArcUsedByRobot[5,2]: w[5,2] - v[5,2] >= 0
 ArcUsedByRobot[5,3]: w[5,3] - v[5,3] >= 0
 ArcUsedByRobot[5,4]: w[5,4] - v[5,4] >= 0
 ArcUsedByRobot[5,5]: w[5,5] - v[5,5] >= 0
 ArcUsedByRobot[5,6]: w[5,6] - v[5,6] >= 0
 ArcUsedByRobot[5,7]: w[5,7] - v[5,7] >= 0
 ArcUsedByRobot[5,8]: w[5,8] - v[5,8] >= 0
 ArcUsedByRobot[5,9]: w[5,9] - v[5,9] >= 0
 ArcUsedByRobot[5,10]: w[5,10] - v[5,10] >= 0
 ArcUsedByRobot[5,11]: w[5,11] - v[5,11] >= 0
 ArcUsedByRobot[5,12]: w[5,12] - v[5,12] >= 0
 ArcUsedByRobot[5,13]: w[5,13] - v[5,13] >= 0
 ArcUsedByRobot[6,1]: w[6,1] - v[6,1] >= 0
 ArcUsedByRobot[6,2]: w[6,2] - v[6,2] >= 0
 ArcUsedByRobot[6,3]: w[6,3] - v[6,3] >= 0
 ArcUsedByRobot[6,4]: w[6,4] - v[6,4] >= 0
 ArcUsedByRobot[6,5]: w[6,5] - v[6,5] >= 0
 ArcUsedByRobot[6,6]: w[6,6] - v[6,6] >= 0
 ArcUsedByRobot[6,7]: w[6,7] - v[6,7] >= 0
 ArcUsedByRobot[6,8]: w[6,8] - v[6,8] >= 0
 ArcUsedByRobot[6,9]: w[6,9] - v[6,9] >= 0
 ArcUsedByRobot[6,10]: w[6,10] - v[6,10] >= 0
 ArcUsedByRobot[6,11]: w[6,11] - v[6,11] >= 0
 ArcUsedByRobot[6,12]: w[6,12] - v[6,12] >= 0
 ArcUsedByRobot[6,13]: w[6,13] - v[6,13] >= 0
 ArcUsedByRobot[7,1]: w[7,1] - v[7,1] >= 0
 ArcUsedByRobot[7,2]: w[7,2] - v[7,2] >= 0
 ArcUsedByRobot[7,3]: w[7,3] - v[7,3] >= 0
 ArcUsedByRobot[7,4]: w[7,4] - v[7,4] >= 0
 ArcUsedByRobot[7,5]: w[7,5] - v[7,5] >= 0
 ArcUsedByRobot[7,6]: w[7,6] - v[7,6] >= 0
 ArcUsedByRobot[7,7]: w[7,7] - v[7,7] >= 0
 ArcUsedByRobot[7,8]: w[7,8] - v[7,8] >= 0
 ArcUsedByRobot[7,9]: w[7,9] - v[7,9] >= 0
 ArcUsedByRobot[7,10]: w[7,10] - v[7,10] >= 0
 ArcUsedByRobot[7,11]: w[7,11] - v[7,11] >= 0
 ArcUsedByRobot[7,12]: w[7,12] - v[7,12] >= 0
 ArcUsedByRobot[7,13]: w[7,13] - v[7,13] >= 0
 ArcUsedByRobot[8,1]: w[8,1] - v[8,1] >= 0
 ArcUsedByRobot[8,2]: w[8,2] - v[8,2] >= 0
 ArcUsedByRobot[8,3]: w[8,3] - v[8,3] >= 0
 ArcUsedByRobot[8,4]: w[8,4] - v[8,4] >= 0
 ArcUsedByRobot[8,5]: w[8,5] - v[8,5] >= 0
 ArcUsedByRobot[8,6]: w[8,6] - v[8,6] >= 0
 ArcUsedByRobot[8,7]: w[8,7] - v[8,7] >= 0
 ArcUsedByRobot[8,8]: w[8,8] - v[8,8] >= 0
 ArcUsedByRobot[8,9]: w[8,9] - v[8,9] >= 0
 ArcUsedByRobot[8,10]: w[8,10] - v[8,10] >= 0
 ArcUsedByRobot[8,11]: w[8,11] - v[8,11] >= 0
 ArcUsedByRobot[8,12]: w[8,12] - v[8,12] >= 0
 ArcUsedByRobot[8,13]: w[8,13] - v[8,13] >= 0
 ArcUsedByRobot[9,1]: w[9,1] - v[9,1] >= 0
 ArcUsedByRobot[9,2]: w[9,2] - v[9,2] >= 0
 ArcUsedByRobot[9,3]: w[9,3] - v[9,3] >= 0
 ArcUsedByRobot[9,4]: w[9,4] - v[9,4] >= 0
 ArcUsedByRobot[9,5]: w[9,5] - v[9,5] >= 0
 ArcUsedByRobot[9,6]: w[9,6] - v[9,6] >= 0
 ArcUsedByRobot[9,7]: w[9,7] - v[9,7] >= 0
 ArcUsedByRobot[9,8]: w[9,8] - v[9,8] >= 0
 ArcUsedByRobot[9,9]: w[9,9] - v[9,9] >= 0
 ArcUsedByRobot[9,10]: w[9,10] - v[9,10] >= 0
 ArcUsedByRobot[9,11]: w[9,11] - v[9,11] >= 0
 ArcUsedByRobot[9,12]: w[9,12] - v[9,12] >= 0
 ArcUsedByRobot[9,13]: w[9,13] - v[9,13] >= 0
 ArcUsedByRobot[10,1]: w[10,1] - v[10,1] >= 0
 ArcUsedByRobot[10,2]: w[10,2] - v[10,2] >= 0
 ArcUsedByRobot[10,3]: w[10,3] - v[10,3] >= 0
 ArcUsedByRobot[10,4]: w[10,4] - v[10,4] >= 0
 ArcUsedByRobot[10,5]: w[10,5] - v[10,5] >= 0
 ArcUsedByRobot[10,6]: w[10,6] - v[10,6] >= 0
 ArcUsedByRobot[10,7]: w[10,7] - v[10,7] >= 0
 ArcUsedByRobot[10,8]: w[10,8] - v[10,8] >= 0
 ArcUsedByRobot[10,9]: w[10,9] - v[10,9] >= 0
 ArcUsedByRobot[10,10]: w[10,10] - v[10,10] >= 0
 ArcUsedByRobot[10,11]: w[10,11] - v[10,11] >= 0
 ArcUsedByRobot[10,12]: w[10,12] - v[10,12] >= 0
 ArcUsedByRobot[10,13]: w[10,13] - v[10,13] >= 0
 ArcUsedByRobot[11,1]: w[11,1] - v[11,1] >= 0
 ArcUsedByRobot[11,2]: w[11,2] - v[11,2] >= 0
 ArcUsedByRobot[11,3]: w[11,3] - v[11,3] >= 0
 ArcUsedByRobot[11,4]: w[11,4] - v[11,4] >= 0
 ArcUsedByRobot[11,5]: w[11,5] - v[11,5] >= 0
 ArcUsedByRobot[11,6]: w[11,6] - v[11,6] >= 0
 ArcUsedByRobot[11,7]: w[11,7] - v[11,7] >= 0
 ArcUsedByRobot[11,8]: w[11,8] - v[11,8] >= 0
 ArcUsedByRobot[11,9]: w[11,9] - v[11,9] >= 0
 ArcUsedByRobot[11,10]: w[11,10] - v[11,10] >= 0
 ArcUsedByRobot[11,11]: w[11,11] - v[11,11] >= 0
 ArcUsedByRobot[11,12]: w[11,12] - v[11,12] >= 0
 ArcUsedByRobot[11,13]: w[11,13] - v[11,13] >= 0
 ArcUsedByRobot[12,1]: w[12,1] - v[12,1] >= 0
 ArcUsedByRobot[12,2]: w[12,2] - v[12,2] >= 0
 ArcUsedByRobot[12,3]: w[12,3] - v[12,3] >= 0
 ArcUsedByRobot[12,4]: w[12,4] - v[12,4] >= 0
 ArcUsedByRobot[12,5]: w[12,5] - v[12,5] >= 0
 ArcUsedByRobot[12,6]: w[12,6] - v[12,6] >= 0
 ArcUsedByRobot[12,7]: w[12,7] - v[12,7] >= 0
 ArcUsedByRobot[12,8]: w[12,8] - v[12,8] >= 0
 ArcUsedByRobot[12,9]: w[12,9] - v[12,9] >= 0
 ArcUsedByRobot[12,10]: w[12,10] - v[12,10] >= 0
 ArcUsedByRobot[12,11]: w[12,11] - v[12,11] >= 0
 ArcUsedByRobot[12,12]: w[12,12] - v[12,12] >= 0
 ArcUsedByRobot[12,13]: w[12,13] - v[12,13] >= 0
 StartServiceRobots[0,1]: - 999999 v[0,1] - service_time[0]
   - start_service[0] + start_service[1] >= -999999
 StartServiceRobots[0,2]: - 999999 v[0,2] - service_time[0]
   - start_service[0] + start_service[2] >= -999954.5446932811
 StartServiceRobots[0,3]: - 999999 v[0,3] - service_time[0]
   - start_service[0] + start_service[3] >= -999934.6400107942
 StartServiceRobots[0,4]: - 999999 v[0,4] - service_time[0]
   - start_service[0] + start_service[4] >= -999968.7774252075
 StartServiceRobots[0,5]: - 999999 v[0,5] - service_time[0]
   - start_service[0] + start_service[5] >= -999946.3820795887
 StartServiceRobots[0,6]: - 999999 v[0,6] - service_time[0]
   - start_service[0] + start_service[6] >= -999945.8158556612
 StartServiceRobots[0,7]: - 999999 v[0,7] - service_time[0]
   - start_service[0] + start_service[7] >= -999954.5446932811
 StartServiceRobots[0,8]: - 999999 v[0,8] - service_time[0]
   - start_service[0] + start_service[8] >= -999934.6400107942
 StartServiceRobots[0,9]: - 999999 v[0,9] - service_time[0]
   - start_service[0] + start_service[9] >= -999968.7774252075
 StartServiceRobots[0,10]: - 999999 v[0,10] - service_time[0]
   - start_service[0] + start_service[10] >= -999946.3820795887
 StartServiceRobots[0,11]: - 999999 v[0,11] - service_time[0]
   - start_service[0] + start_service[11] >= -999945.8158556612
 StartServiceRobots[0,12]: - 999999 v[0,12] - service_time[0]
   - start_service[0] + start_service[12] >= -999999
 StartServiceRobots[0,13]: - 999999 v[0,13] - service_time[0]
   - start_service[0] + start_service[13] >= -999999
 StartServiceRobots[1,1]: - 999999 v[1,1] - service_time[1] >= -999999
 StartServiceRobots[1,2]: - 999999 v[1,2] - service_time[1]
   - start_service[1] + start_service[2] >= -999954.5446932811
 StartServiceRobots[1,3]: - 999999 v[1,3] - service_time[1]
   - start_service[1] + start_service[3] >= -999934.6400107942
 StartServiceRobots[1,4]: - 999999 v[1,4] - service_time[1]
   - start_service[1] + start_service[4] >= -999968.7774252075
 StartServiceRobots[1,5]: - 999999 v[1,5] - service_time[1]
   - start_service[1] + start_service[5] >= -999946.3820795887
 StartServiceRobots[1,6]: - 999999 v[1,6] - service_time[1]
   - start_service[1] + start_service[6] >= -999945.8158556612
 StartServiceRobots[1,7]: - 999999 v[1,7] - service_time[1]
   - start_service[1] + start_service[7] >= -999954.5446932811
 StartServiceRobots[1,8]: - 999999 v[1,8] - service_time[1]
   - start_service[1] + start_service[8] >= -999934.6400107942
 StartServiceRobots[1,9]: - 999999 v[1,9] - service_time[1]
   - start_service[1] + start_service[9] >= -999968.7774252075
 StartServiceRobots[1,10]: - 999999 v[1,10] - service_time[1]
   - start_service[1] + start_service[10] >= -999946.3820795887
 StartServiceRobots[1,11]: - 999999 v[1,11] - service_time[1]
   - start_service[1] + start_service[11] >= -999945.8158556612
 StartServiceRobots[1,12]: - 999999 v[1,12] - service_time[1]
   - start_service[1] + start_service[12] >= -999999
 StartServiceRobots[1,13]: - 999999 v[1,13] - service_time[1]
   - start_service[1] + start_service[13] >= -999999
 StartServiceRobots[2,1]: - 999999 v[2,1] - service_time[2]
   + start_service[1] - start_service[2] >= -999954.5446932811
 StartServiceRobots[2,2]: - 999999 v[2,2] - service_time[2] >= -999999
 StartServiceRobots[2,3]: - 999999 v[2,3] - service_time[2]
   - start_service[2] + start_service[3] >= -999900.8745887573
 StartServiceRobots[2,4]: - 999999 v[2,4] - service_time[2]
   - start_service[2] + start_service[4] >= -999943.5901960946
 StartServiceRobots[2,5]: - 999999 v[2,5] - service_time[2]
   - start_service[2] + start_service[5] >= -999904.5210086526
 StartServiceRobots[2,6]: - 999999 v[2,6] - service_time[2]
   - start_service[2] + start_service[6] >= -999912.5492633492
 StartServiceRobots[2,7]: - 999999 v[2,7] - service_time[2]
   - start_service[2] + start_service[7] >= -999999
 StartServiceRobots[2,8]: - 999999 v[2,8] - service_time[2]
   - start_service[2] + start_service[8] >= -999900.8745887573
 StartServiceRobots[2,9]: - 999999 v[2,9] - service_time[2]
   - start_service[2] + start_service[9] >= -999943.5901960946
 StartServiceRobots[2,10]: - 999999 v[2,10] - service_time[2]
   - start_service[2] + start_service[10] >= -999904.5210086526
 StartServiceRobots[2,11]: - 999999 v[2,11] - service_time[2]
   - start_service[2] + start_service[11] >= -999912.5492633492
 StartServiceRobots[2,12]: - 999999 v[2,12] - service_time[2]
   - start_service[2] + start_service[12] >= -999954.5446932811
 StartServiceRobots[2,13]: - 999999 v[2,13] - service_time[2]
   - start_service[2] + start_service[13] >= -999954.5446932811
 StartServiceRobots[3,1]: - 999999 v[3,1] - service_time[3]
   + start_service[1] - start_service[3] >= -999934.6400107942
 StartServiceRobots[3,2]: - 999999 v[3,2] - service_time[3]
   + start_service[2] - start_service[3] >= -999900.8745887573
 StartServiceRobots[3,3]: - 999999 v[3,3] - service_time[3] >= -999999
 StartServiceRobots[3,4]: - 999999 v[3,4] - service_time[3]
   - start_service[3] + start_service[4] >= -999956.223568582
 StartServiceRobots[3,5]: - 999999 v[3,5] - service_time[3]
   - start_service[3] + start_service[5] >= -999924.1893788762
 StartServiceRobots[3,6]: - 999999 v[3,6] - service_time[3]
   - start_service[3] + start_service[6] >= -999903.7963930037
 StartServiceRobots[3,7]: - 999999 v[3,7] - service_time[3]
   - start_service[3] + start_service[7] >= -999900.8745887573
 StartServiceRobots[3,8]: - 999999 v[3,8] - service_time[3]
   - start_service[3] + start_service[8] >= -999999
 StartServiceRobots[3,9]: - 999999 v[3,9] - service_time[3]
   - start_service[3] + start_service[9] >= -999956.223568582
 StartServiceRobots[3,10]: - 999999 v[3,10] - service_time[3]
   - start_service[3] + start_service[10] >= -999924.1893788762
 StartServiceRobots[3,11]: - 999999 v[3,11] - service_time[3]
   - start_service[3] + start_service[11] >= -999903.7963930037
 StartServiceRobots[3,12]: - 999999 v[3,12] - service_time[3]
   - start_service[3] + start_service[12] >= -999934.6400107942
 StartServiceRobots[3,13]: - 999999 v[3,13] - service_time[3]
   - start_service[3] + start_service[13] >= -999934.6400107942
 StartServiceRobots[4,1]: - 999999 v[4,1] - service_time[4]
   + start_service[1] - start_service[4] >= -999968.7774252075
 StartServiceRobots[4,2]: - 999999 v[4,2] - service_time[4]
   + start_service[2] - start_service[4] >= -999943.5901960946
 StartServiceRobots[4,3]: - 999999 v[4,3] - service_time[4]
   + start_service[3] - start_service[4] >= -999956.223568582
 StartServiceRobots[4,4]: - 999999 v[4,4] - service_time[4] >= -999999
 StartServiceRobots[4,5]: - 999999 v[4,5] - service_time[4]
   - start_service[4] + start_service[5] >= -999928.8980723232
 StartServiceRobots[4,6]: - 999999 v[4,6] - service_time[4]
   - start_service[4] + start_service[6] >= -999919.8307914401
 StartServiceRobots[4,7]: - 999999 v[4,7] - service_time[4]
   - start_service[4] + start_service[7] >= -999943.5901960946
 StartServiceRobots[4,8]: - 999999 v[4,8] - service_time[4]
   - start_service[4] + start_service[8] >= -999956.223568582
 StartServiceRobots[4,9]: - 999999 v[4,9] - service_time[4]
   - start_service[4] + start_service[9] >= -999999
 StartServiceRobots[4,10]: - 999999 v[4,10] - service_time[4]
   - start_service[4] + start_service[10] >= -999928.8980723232
 StartServiceRobots[4,11]: - 999999 v[4,11] - service_time[4]
   - start_service[4] + start_service[11] >= -999919.8307914401
 StartServiceRobots[4,12]: - 999999 v[4,12] - service_time[4]
   - start_service[4] + start_service[12] >= -999968.7774252075
 StartServiceRobots[4,13]: - 999999 v[4,13] - service_time[4]
   - start_service[4] + start_service[13] >= -999968.7774252075
 StartServiceRobots[5,1]: - 999999 v[5,1] - service_time[5]
   + start_service[1] - start_service[5] >= -999946.3820795887
 StartServiceRobots[5,2]: - 999999 v[5,2] - service_time[5]
   + start_service[2] - start_service[5] >= -999904.5210086526
 StartServiceRobots[5,3]: - 999999 v[5,3] - service_time[5]
   + start_service[3] - start_service[5] >= -999924.1893788762
 StartServiceRobots[5,4]: - 999999 v[5,4] - service_time[5]
   + start_service[4] - start_service[5] >= -999928.8980723232
 StartServiceRobots[5,5]: - 999999 v[5,5] - service_time[5] >= -999999
 StartServiceRobots[5,6]: - 999999 v[5,6] - service_time[5]
   - start_service[5] + start_service[6] >= -999972.5120726503
 StartServiceRobots[5,7]: - 999999 v[5,7] - service_time[5]
   - start_service[5] + start_service[7] >= -999904.5210086526
 StartServiceRobots[5,8]: - 999999 v[5,8] - service_time[5]
   - start_service[5] + start_service[8] >= -999924.1893788762
 StartServiceRobots[5,9]: - 999999 v[5,9] - service_time[5]
   - start_service[5] + start_service[9] >= -999928.8980723232
 StartServiceRobots[5,10]: - 999999 v[5,10] - service_time[5]
   - start_service[5] + start_service[10] >= -999999
 StartServiceRobots[5,11]: - 999999 v[5,11] - service_time[5]
   - start_service[5] + start_service[11] >= -999972.5120726503
 StartServiceRobots[5,12]: - 999999 v[5,12] - service_time[5]
   - start_service[5] + start_service[12] >= -999946.3820795887
 StartServiceRobots[5,13]: - 999999 v[5,13] - service_time[5]
   - start_service[5] + start_service[13] >= -999946.3820795887
 StartServiceRobots[6,1]: - 999999 v[6,1] - service_time[6]
   + start_service[1] - start_service[6] >= -999945.8158556612
 StartServiceRobots[6,2]: - 999999 v[6,2] - service_time[6]
   + start_service[2] - start_service[6] >= -999912.5492633492
 StartServiceRobots[6,3]: - 999999 v[6,3] - service_time[6]
   + start_service[3] - start_service[6] >= -999903.7963930037
 StartServiceRobots[6,4]: - 999999 v[6,4] - service_time[6]
   + start_service[4] - start_service[6] >= -999919.8307914401
 StartServiceRobots[6,5]: - 999999 v[6,5] - service_time[6]
   + start_service[5] - start_service[6] >= -999972.5120726503
 StartServiceRobots[6,6]: - 999999 v[6,6] - service_time[6] >= -999999
 StartServiceRobots[6,7]: - 999999 v[6,7] - service_time[6]
   - start_service[6] + start_service[7] >= -999912.5492633492
 StartServiceRobots[6,8]: - 999999 v[6,8] - service_time[6]
   - start_service[6] + start_service[8] >= -999903.7963930037
 StartServiceRobots[6,9]: - 999999 v[6,9] - service_time[6]
   - start_service[6] + start_service[9] >= -999919.8307914401
 StartServiceRobots[6,10]: - 999999 v[6,10] - service_time[6]
   - start_service[6] + start_service[10] >= -999972.5120726503
 StartServiceRobots[6,11]: - 999999 v[6,11] - service_time[6]
   - start_service[6] + start_service[11] >= -999999
 StartServiceRobots[6,12]: - 999999 v[6,12] - service_time[6]
   - start_service[6] + start_service[12] >= -999945.8158556612
 StartServiceRobots[6,13]: - 999999 v[6,13] - service_time[6]
   - start_service[6] + start_service[13] >= -999945.8158556612
 StartServiceRobots[7,1]: - 999999 v[7,1] - service_time[7]
   + start_service[1] - start_service[7] >= -999954.5446932811
 StartServiceRobots[7,2]: - 999999 v[7,2] - service_time[7]
   + start_service[2] - start_service[7] >= -999999
 StartServiceRobots[7,3]: - 999999 v[7,3] - service_time[7]
   + start_service[3] - start_service[7] >= -999900.8745887573
 StartServiceRobots[7,4]: - 999999 v[7,4] - service_time[7]
   + start_service[4] - start_service[7] >= -999943.5901960946
 StartServiceRobots[7,5]: - 999999 v[7,5] - service_time[7]
   + start_service[5] - start_service[7] >= -999904.5210086526
 StartServiceRobots[7,6]: - 999999 v[7,6] - service_time[7]
   + start_service[6] - start_service[7] >= -999912.5492633492
 StartServiceRobots[7,7]: - 999999 v[7,7] - service_time[7] >= -999999
 StartServiceRobots[7,8]: - 999999 v[7,8] - service_time[7]
   - start_service[7] + start_service[8] >= -999900.8745887573
 StartServiceRobots[7,9]: - 999999 v[7,9] - service_time[7]
   - start_service[7] + start_service[9] >= -999943.5901960946
 StartServiceRobots[7,10]: - 999999 v[7,10] - service_time[7]
   - start_service[7] + start_service[10] >= -999904.5210086526
 StartServiceRobots[7,11]: - 999999 v[7,11] - service_time[7]
   - start_service[7] + start_service[11] >= -999912.5492633492
 StartServiceRobots[7,12]: - 999999 v[7,12] - service_time[7]
   - start_service[7] + start_service[12] >= -999954.5446932811
 StartServiceRobots[7,13]: - 999999 v[7,13] - service_time[7]
   - start_service[7] + start_service[13] >= -999954.5446932811
 StartServiceRobots[8,1]: - 999999 v[8,1] - service_time[8]
   + start_service[1] - start_service[8] >= -999934.6400107942
 StartServiceRobots[8,2]: - 999999 v[8,2] - service_time[8]
   + start_service[2] - start_service[8] >= -999900.8745887573
 StartServiceRobots[8,3]: - 999999 v[8,3] - service_time[8]
   + start_service[3] - start_service[8] >= -999999
 StartServiceRobots[8,4]: - 999999 v[8,4] - service_time[8]
   + start_service[4] - start_service[8] >= -999956.223568582
 StartServiceRobots[8,5]: - 999999 v[8,5] - service_time[8]
   + start_service[5] - start_service[8] >= -999924.1893788762
 StartServiceRobots[8,6]: - 999999 v[8,6] - service_time[8]
   + start_service[6] - start_service[8] >= -999903.7963930037
 StartServiceRobots[8,7]: - 999999 v[8,7] - service_time[8]
   + start_service[7] - start_service[8] >= -999900.8745887573
 StartServiceRobots[8,8]: - 999999 v[8,8] - service_time[8] >= -999999
 StartServiceRobots[8,9]: - 999999 v[8,9] - service_time[8]
   - start_service[8] + start_service[9] >= -999956.223568582
 StartServiceRobots[8,10]: - 999999 v[8,10] - service_time[8]
   - start_service[8] + start_service[10] >= -999924.1893788762
 StartServiceRobots[8,11]: - 999999 v[8,11] - service_time[8]
   - start_service[8] + start_service[11] >= -999903.7963930037
 StartServiceRobots[8,12]: - 999999 v[8,12] - service_time[8]
   - start_service[8] + start_service[12] >= -999934.6400107942
 StartServiceRobots[8,13]: - 999999 v[8,13] - service_time[8]
   - start_service[8] + start_service[13] >= -999934.6400107942
 StartServiceRobots[9,1]: - 999999 v[9,1] - service_time[9]
   + start_service[1] - start_service[9] >= -999968.7774252075
 StartServiceRobots[9,2]: - 999999 v[9,2] - service_time[9]
   + start_service[2] - start_service[9] >= -999943.5901960946
 StartServiceRobots[9,3]: - 999999 v[9,3] - service_time[9]
   + start_service[3] - start_service[9] >= -999956.223568582
 StartServiceRobots[9,4]: - 999999 v[9,4] - service_time[9]
   + start_service[4] - start_service[9] >= -999999
 StartServiceRobots[9,5]: - 999999 v[9,5] - service_time[9]
   + start_service[5] - start_service[9] >= -999928.8980723232
 StartServiceRobots[9,6]: - 999999 v[9,6] - service_time[9]
   + start_service[6] - start_service[9] >= -999919.8307914401
 StartServiceRobots[9,7]: - 999999 v[9,7] - service_time[9]
   + start_service[7] - start_service[9] >= -999943.5901960946
 StartServiceRobots[9,8]: - 999999 v[9,8] - service_time[9]
   + start_service[8] - start_service[9] >= -999956.223568582
 StartServiceRobots[9,9]: - 999999 v[9,9] - service_time[9] >= -999999
 StartServiceRobots[9,10]: - 999999 v[9,10] - service_time[9]
   - start_service[9] + start_service[10] >= -999928.8980723232
 StartServiceRobots[9,11]: - 999999 v[9,11] - service_time[9]
   - start_service[9] + start_service[11] >= -999919.8307914401
 StartServiceRobots[9,12]: - 999999 v[9,12] - service_time[9]
   - start_service[9] + start_service[12] >= -999968.7774252075
 StartServiceRobots[9,13]: - 999999 v[9,13] - service_time[9]
   - start_service[9] + start_service[13] >= -999968.7774252075
 StartServiceRobots[10,1]: - 999999 v[10,1] - service_time[10]
   + start_service[1] - start_service[10] >= -999946.3820795887
 StartServiceRobots[10,2]: - 999999 v[10,2] - service_time[10]
   + start_service[2] - start_service[10] >= -999904.5210086526
 StartServiceRobots[10,3]: - 999999 v[10,3] - service_time[10]
   + start_service[3] - start_service[10] >= -999924.1893788762
 StartServiceRobots[10,4]: - 999999 v[10,4] - service_time[10]
   + start_service[4] - start_service[10] >= -999928.8980723232
 StartServiceRobots[10,5]: - 999999 v[10,5] - service_time[10]
   + start_service[5] - start_service[10] >= -999999
 StartServiceRobots[10,6]: - 999999 v[10,6] - service_time[10]
   + start_service[6] - start_service[10] >= -999972.5120726503
 StartServiceRobots[10,7]: - 999999 v[10,7] - service_time[10]
   + start_service[7] - start_service[10] >= -999904.5210086526
 StartServiceRobots[10,8]: - 999999 v[10,8] - service_time[10]
   + start_service[8] - start_service[10] >= -999924.1893788762
 StartServiceRobots[10,9]: - 999999 v[10,9] - service_time[10]
   + start_service[9] - start_service[10] >= -999928.8980723232
 StartServiceRobots[10,10]: - 999999 v[10,10] - service_time[10]
   >= -999999
 StartServiceRobots[10,11]: - 999999 v[10,11] - service_time[10]
   - start_service[10] + start_service[11] >= -999972.5120726503
 StartServiceRobots[10,12]: - 999999 v[10,12] - service_time[10]
   - start_service[10] + start_service[12] >= -999946.3820795887
 StartServiceRobots[10,13]: - 999999 v[10,13] - service_time[10]
   - start_service[10] + start_service[13] >= -999946.3820795887
 StartServiceRobots[11,1]: - 999999 v[11,1] - service_time[11]
   + start_service[1] - start_service[11] >= -999945.8158556612
 StartServiceRobots[11,2]: - 999999 v[11,2] - service_time[11]
   + start_service[2] - start_service[11] >= -999912.5492633492
 StartServiceRobots[11,3]: - 999999 v[11,3] - service_time[11]
   + start_service[3] - start_service[11] >= -999903.7963930037
 StartServiceRobots[11,4]: - 999999 v[11,4] - service_time[11]
   + start_service[4] - start_service[11] >= -999919.8307914401
 StartServiceRobots[11,5]: - 999999 v[11,5] - service_time[11]
   + start_service[5] - start_service[11] >= -999972.5120726503
 StartServiceRobots[11,6]: - 999999 v[11,6] - service_time[11]
   + start_service[6] - start_service[11] >= -999999
 StartServiceRobots[11,7]: - 999999 v[11,7] - service_time[11]
   + start_service[7] - start_service[11] >= -999912.5492633492
 StartServiceRobots[11,8]: - 999999 v[11,8] - service_time[11]
   + start_service[8] - start_service[11] >= -999903.7963930037
 StartServiceRobots[11,9]: - 999999 v[11,9] - service_time[11]
   + start_service[9] - start_service[11] >= -999919.8307914401
 StartServiceRobots[11,10]: - 999999 v[11,10] - service_time[11]
   + start_service[10] - start_service[11] >= -999972.5120726503
 StartServiceRobots[11,11]: - 999999 v[11,11] - service_time[11]
   >= -999999
 StartServiceRobots[11,12]: - 999999 v[11,12] - service_time[11]
   - start_service[11] + start_service[12] >= -999945.8158556612
 StartServiceRobots[11,13]: - 999999 v[11,13] - service_time[11]
   - start_service[11] + start_service[13] >= -999945.8158556612
 StartServiceRobots[12,1]: - 999999 v[12,1] - service_time[12]
   + start_service[1] - start_service[12] >= -999999
 StartServiceRobots[12,2]: - 999999 v[12,2] - service_time[12]
   + start_service[2] - start_service[12] >= -999954.5446932811
 StartServiceRobots[12,3]: - 999999 v[12,3] - service_time[12]
   + start_service[3] - start_service[12] >= -999934.6400107942
 StartServiceRobots[12,4]: - 999999 v[12,4] - service_time[12]
   + start_service[4] - start_service[12] >= -999968.7774252075
 StartServiceRobots[12,5]: - 999999 v[12,5] - service_time[12]
   + start_service[5] - start_service[12] >= -999946.3820795887
 StartServiceRobots[12,6]: - 999999 v[12,6] - service_time[12]
   + start_service[6] - start_service[12] >= -999945.8158556612
 StartServiceRobots[12,7]: - 999999 v[12,7] - service_time[12]
   + start_service[7] - start_service[12] >= -999954.5446932811
 StartServiceRobots[12,8]: - 999999 v[12,8] - service_time[12]
   + start_service[8] - start_service[12] >= -999934.6400107942
 StartServiceRobots[12,9]: - 999999 v[12,9] - service_time[12]
   + start_service[9] - start_service[12] >= -999968.7774252075
 StartServiceRobots[12,10]: - 999999 v[12,10] - service_time[12]
   + start_service[10] - start_service[12] >= -999946.3820795887
 StartServiceRobots[12,11]: - 999999 v[12,11] - service_time[12]
   + start_service[11] - start_service[12] >= -999945.8158556612
 StartServiceRobots[12,12]: - 999999 v[12,12] - service_time[12]
   >= -999999
 StartServiceRobots[12,13]: - 999999 v[12,13] - service_time[12]
   - start_service[12] + start_service[13] >= -999999
 RobotsEndService[2]: - service_time[2] - start_service[2]
   + start_service[7] >= 0
 RobotsEndService[3]: - service_time[3] - start_service[3]
   + start_service[8] >= 0
 RobotsEndService[4]: - service_time[4] - start_service[4]
   + start_service[9] >= 0
 RobotsEndService[5]: - service_time[5] - start_service[5]
   + start_service[10] >= 0
 RobotsEndService[6]: - service_time[6] - start_service[6]
   + start_service[11] >= 0
 VehicleRobotSynchronization1[0,1]: - 2 x[0,1] + w[0,1] <= 0
 VehicleRobotSynchronization1[0,2]: - 2 x[0,2] + w[0,2] <= 0
 VehicleRobotSynchronization1[0,3]: - 2 x[0,3] + w[0,3] <= 0
 VehicleRobotSynchronization1[0,4]: - 2 x[0,4] + w[0,4] <= 0
 VehicleRobotSynchronization1[0,5]: - 2 x[0,5] + w[0,5] <= 0
 VehicleRobotSynchronization1[0,6]: - 2 x[0,6] + w[0,6] <= 0
 VehicleRobotSynchronization1[0,7]: - 2 x[0,7] + w[0,7] <= 0
 VehicleRobotSynchronization1[0,8]: - 2 x[0,8] + w[0,8] <= 0
 VehicleRobotSynchronization1[0,9]: - 2 x[0,9] + w[0,9] <= 0
 VehicleRobotSynchronization1[0,10]: - 2 x[0,10] + w[0,10] <= 0
 VehicleRobotSynchronization1[0,11]: - 2 x[0,11] + w[0,11] <= 0
 VehicleRobotSynchronization1[0,12]: - 2 x[0,12] + w[0,12] <= 0
 VehicleRobotSynchronization1[0,13]: - 2 x[0,13] + w[0,13] <= 0
 VehicleRobotSynchronization1[1,0]: - 2 x[1,0] + w[1,0] <= 0
 VehicleRobotSynchronization1[1,2]: - 2 x[1,2] + w[1,2] <= 0
 VehicleRobotSynchronization1[1,3]: - 2 x[1,3] + w[1,3] <= 0
 VehicleRobotSynchronization1[1,4]: - 2 x[1,4] + w[1,4] <= 0
 VehicleRobotSynchronization1[1,5]: - 2 x[1,5] + w[1,5] <= 0
 VehicleRobotSynchronization1[1,6]: - 2 x[1,6] + w[1,6] <= 0
 VehicleRobotSynchronization1[1,7]: - 2 x[1,7] + w[1,7] <= 0
 VehicleRobotSynchronization1[1,8]: - 2 x[1,8] + w[1,8] <= 0
 VehicleRobotSynchronization1[1,9]: - 2 x[1,9] + w[1,9] <= 0
 VehicleRobotSynchronization1[1,10]: - 2 x[1,10] + w[1,10] <= 0
 VehicleRobotSynchronization1[1,11]: - 2 x[1,11] + w[1,11] <= 0
 VehicleRobotSynchronization1[1,12]: - 2 x[1,12] + w[1,12] <= 0
 VehicleRobotSynchronization1[1,13]: - 2 x[1,13] + w[1,13] <= 0
 VehicleRobotSynchronization1[2,0]: - 2 x[2,0] + w[2,0] <= 0
 VehicleRobotSynchronization1[2,1]: - 2 x[2,1] + w[2,1] <= 0
 VehicleRobotSynchronization1[2,3]: - 2 x[2,3] + w[2,3] <= 0
 VehicleRobotSynchronization1[2,4]: - 2 x[2,4] + w[2,4] <= 0
 VehicleRobotSynchronization1[2,5]: - 2 x[2,5] + w[2,5] <= 0
 VehicleRobotSynchronization1[2,6]: - 2 x[2,6] + w[2,6] <= 0
 VehicleRobotSynchronization1[2,8]: - 2 x[2,8] + w[2,8] <= 0
 VehicleRobotSynchronization1[2,9]: - 2 x[2,9] + w[2,9] <= 0
 VehicleRobotSynchronization1[2,10]: - 2 x[2,10] + w[2,10] <= 0
 VehicleRobotSynchronization1[2,11]: - 2 x[2,11] + w[2,11] <= 0
 VehicleRobotSynchronization1[2,12]: - 2 x[2,12] + w[2,12] <= 0
 VehicleRobotSynchronization1[2,13]: - 2 x[2,13] + w[2,13] <= 0
 VehicleRobotSynchronization1[3,0]: - 2 x[3,0] + w[3,0] <= 0
 VehicleRobotSynchronization1[3,1]: - 2 x[3,1] + w[3,1] <= 0
 VehicleRobotSynchronization1[3,2]: - 2 x[3,2] + w[3,2] <= 0
 VehicleRobotSynchronization1[3,4]: - 2 x[3,4] + w[3,4] <= 0
 VehicleRobotSynchronization1[3,5]: - 2 x[3,5] + w[3,5] <= 0
 VehicleRobotSynchronization1[3,6]: - 2 x[3,6] + w[3,6] <= 0
 VehicleRobotSynchronization1[3,7]: - 2 x[3,7] + w[3,7] <= 0
 VehicleRobotSynchronization1[3,9]: - 2 x[3,9] + w[3,9] <= 0
 VehicleRobotSynchronization1[3,10]: - 2 x[3,10] + w[3,10] <= 0
 VehicleRobotSynchronization1[3,11]: - 2 x[3,11] + w[3,11] <= 0
 VehicleRobotSynchronization1[3,12]: - 2 x[3,12] + w[3,12] <= 0
 VehicleRobotSynchronization1[3,13]: - 2 x[3,13] + w[3,13] <= 0
 VehicleRobotSynchronization1[4,0]: - 2 x[4,0] + w[4,0] <= 0
 VehicleRobotSynchronization1[4,1]: - 2 x[4,1] + w[4,1] <= 0
 VehicleRobotSynchronization1[4,2]: - 2 x[4,2] + w[4,2] <= 0
 VehicleRobotSynchronization1[4,3]: - 2 x[4,3] + w[4,3] <= 0
 VehicleRobotSynchronization1[4,5]: - 2 x[4,5] + w[4,5] <= 0
 VehicleRobotSynchronization1[4,6]: - 2 x[4,6] + w[4,6] <= 0
 VehicleRobotSynchronization1[4,7]: - 2 x[4,7] + w[4,7] <= 0
 VehicleRobotSynchronization1[4,8]: - 2 x[4,8] + w[4,8] <= 0
 VehicleRobotSynchronization1[4,10]: - 2 x[4,10] + w[4,10] <= 0
 VehicleRobotSynchronization1[4,11]: - 2 x[4,11] + w[4,11] <= 0
 VehicleRobotSynchronization1[4,12]: - 2 x[4,12] + w[4,12] <= 0
 VehicleRobotSynchronization1[4,13]: - 2 x[4,13] + w[4,13] <= 0
 VehicleRobotSynchronization1[5,0]: - 2 x[5,0] + w[5,0] <= 0
 VehicleRobotSynchronization1[5,1]: - 2 x[5,1] + w[5,1] <= 0
 VehicleRobotSynchronization1[5,2]: - 2 x[5,2] + w[5,2] <= 0
 VehicleRobotSynchronization1[5,3]: - 2 x[5,3] + w[5,3] <= 0
 VehicleRobotSynchronization1[5,4]: - 2 x[5,4] + w[5,4] <= 0
 VehicleRobotSynchronization1[5,6]: - 2 x[5,6] + w[5,6] <= 0
 VehicleRobotSynchronization1[5,7]: - 2 x[5,7] + w[5,7] <= 0
 VehicleRobotSynchronization1[5,8]: - 2 x[5,8] + w[5,8] <= 0
 VehicleRobotSynchronization1[5,9]: - 2 x[5,9] + w[5,9] <= 0
 VehicleRobotSynchronization1[5,11]: - 2 x[5,11] + w[5,11] <= 0
 VehicleRobotSynchronization1[5,12]: - 2 x[5,12] + w[5,12] <= 0
 VehicleRobotSynchronization1[5,13]: - 2 x[5,13] + w[5,13] <= 0
 VehicleRobotSynchronization1[6,0]: - 2 x[6,0] + w[6,0] <= 0
 VehicleRobotSynchronization1[6,1]: - 2 x[6,1] + w[6,1] <= 0
 VehicleRobotSynchronization1[6,2]: - 2 x[6,2] + w[6,2] <= 0
 VehicleRobotSynchronization1[6,3]: - 2 x[6,3] + w[6,3] <= 0
 VehicleRobotSynchronization1[6,4]: - 2 x[6,4] + w[6,4] <= 0
 VehicleRobotSynchronization1[6,5]: - 2 x[6,5] + w[6,5] <= 0
 VehicleRobotSynchronization1[6,7]: - 2 x[6,7] + w[6,7] <= 0
 VehicleRobotSynchronization1[6,8]: - 2 x[6,8] + w[6,8] <= 0
 VehicleRobotSynchronization1[6,9]: - 2 x[6,9] + w[6,9] <= 0
 VehicleRobotSynchronization1[6,10]: - 2 x[6,10] + w[6,10] <= 0
 VehicleRobotSynchronization1[6,12]: - 2 x[6,12] + w[6,12] <= 0
 VehicleRobotSynchronization1[6,13]: - 2 x[6,13] + w[6,13] <= 0
 VehicleRobotSynchronization1[7,0]: - 2 x[7,0] + w[7,0] <= 0
 VehicleRobotSynchronization1[7,1]: - 2 x[7,1] + w[7,1] <= 0
 VehicleRobotSynchronization1[7,3]: - 2 x[7,3] + w[7,3] <= 0
 VehicleRobotSynchronization1[7,4]: - 2 x[7,4] + w[7,4] <= 0
 VehicleRobotSynchronization1[7,5]: - 2 x[7,5] + w[7,5] <= 0
 VehicleRobotSynchronization1[7,6]: - 2 x[7,6] + w[7,6] <= 0
 VehicleRobotSynchronization1[7,8]: - 2 x[7,8] + w[7,8] <= 0
 VehicleRobotSynchronization1[7,9]: - 2 x[7,9] + w[7,9] <= 0
 VehicleRobotSynchronization1[7,10]: - 2 x[7,10] + w[7,10] <= 0
 VehicleRobotSynchronization1[7,11]: - 2 x[7,11] + w[7,11] <= 0
 VehicleRobotSynchronization1[7,12]: - 2 x[7,12] + w[7,12] <= 0
 VehicleRobotSynchronization1[7,13]: - 2 x[7,13] + w[7,13] <= 0
 VehicleRobotSynchronization1[8,0]: - 2 x[8,0] + w[8,0] <= 0
 VehicleRobotSynchronization1[8,1]: - 2 x[8,1] + w[8,1] <= 0
 VehicleRobotSynchronization1[8,2]: - 2 x[8,2] + w[8,2] <= 0
 VehicleRobotSynchronization1[8,4]: - 2 x[8,4] + w[8,4] <= 0
 VehicleRobotSynchronization1[8,5]: - 2 x[8,5] + w[8,5] <= 0
 VehicleRobotSynchronization1[8,6]: - 2 x[8,6] + w[8,6] <= 0
 VehicleRobotSynchronization1[8,7]: - 2 x[8,7] + w[8,7] <= 0
 VehicleRobotSynchronization1[8,9]: - 2 x[8,9] + w[8,9] <= 0
 VehicleRobotSynchronization1[8,10]: - 2 x[8,10] + w[8,10] <= 0
 VehicleRobotSynchronization1[8,11]: - 2 x[8,11] + w[8,11] <= 0
 VehicleRobotSynchronization1[8,12]: - 2 x[8,12] + w[8,12] <= 0
 VehicleRobotSynchronization1[8,13]: - 2 x[8,13] + w[8,13] <= 0
 VehicleRobotSynchronization1[9,0]: - 2 x[9,0] + w[9,0] <= 0
 VehicleRobotSynchronization1[9,1]: - 2 x[9,1] + w[9,1] <= 0
 VehicleRobotSynchronization1[9,2]: - 2 x[9,2] + w[9,2] <= 0
 VehicleRobotSynchronization1[9,3]: - 2 x[9,3] + w[9,3] <= 0
 VehicleRobotSynchronization1[9,5]: - 2 x[9,5] + w[9,5] <= 0
 VehicleRobotSynchronization1[9,6]: - 2 x[9,6] + w[9,6] <= 0
 VehicleRobotSynchronization1[9,7]: - 2 x[9,7] + w[9,7] <= 0
 VehicleRobotSynchronization1[9,8]: - 2 x[9,8] + w[9,8] <= 0
 VehicleRobotSynchronization1[9,10]: - 2 x[9,10] + w[9,10] <= 0
 VehicleRobotSynchronization1[9,11]: - 2 x[9,11] + w[9,11] <= 0
 VehicleRobotSynchronization1[9,12]: - 2 x[9,12] + w[9,12] <= 0
 VehicleRobotSynchronization1[9,13]: - 2 x[9,13] + w[9,13] <= 0
 VehicleRobotSynchronization1[10,0]: - 2 x[10,0] + w[10,0] <= 0
 VehicleRobotSynchronization1[10,1]: - 2 x[10,1] + w[10,1] <= 0
 VehicleRobotSynchronization1[10,2]: - 2 x[10,2] + w[10,2] <= 0
 VehicleRobotSynchronization1[10,3]: - 2 x[10,3] + w[10,3] <= 0
 VehicleRobotSynchronization1[10,4]: - 2 x[10,4] + w[10,4] <= 0
 VehicleRobotSynchronization1[10,6]: - 2 x[10,6] + w[10,6] <= 0
 VehicleRobotSynchronization1[10,7]: - 2 x[10,7] + w[10,7] <= 0
 VehicleRobotSynchronization1[10,8]: - 2 x[10,8] + w[10,8] <= 0
 VehicleRobotSynchronization1[10,9]: - 2 x[10,9] + w[10,9] <= 0
 VehicleRobotSynchronization1[10,11]: - 2 x[10,11] + w[10,11] <= 0
 VehicleRobotSynchronization1[10,12]: - 2 x[10,12] + w[10,12] <= 0
 VehicleRobotSynchronization1[10,13]: - 2 x[10,13] + w[10,13] <= 0
 VehicleRobotSynchronization1[11,0]: - 2 x[11,0] + w[11,0] <= 0
 VehicleRobotSynchronization1[11,1]: - 2 x[11,1] + w[11,1] <= 0
 VehicleRobotSynchronization1[11,2]: - 2 x[11,2] + w[11,2] <= 0
 VehicleRobotSynchronization1[11,3]: - 2 x[11,3] + w[11,3] <= 0
 VehicleRobotSynchronization1[11,4]: - 2 x[11,4] + w[11,4] <= 0
 VehicleRobotSynchronization1[11,5]: - 2 x[11,5] + w[11,5] <= 0
 VehicleRobotSynchronization1[11,7]: - 2 x[11,7] + w[11,7] <= 0
 VehicleRobotSynchronization1[11,8]: - 2 x[11,8] + w[11,8] <= 0
 VehicleRobotSynchronization1[11,9]: - 2 x[11,9] + w[11,9] <= 0
 VehicleRobotSynchronization1[11,10]: - 2 x[11,10] + w[11,10] <= 0
 VehicleRobotSynchronization1[11,12]: - 2 x[11,12] + w[11,12] <= 0
 VehicleRobotSynchronization1[11,13]: - 2 x[11,13] + w[11,13] <= 0
 VehicleRobotSynchronization1[12,0]: - 2 x[12,0] + w[12,0] <= 0
 VehicleRobotSynchronization1[12,1]: - 2 x[12,1] + w[12,1] <= 0
 VehicleRobotSynchronization1[12,2]: - 2 x[12,2] + w[12,2] <= 0
 VehicleRobotSynchronization1[12,3]: - 2 x[12,3] + w[12,3] <= 0
 VehicleRobotSynchronization1[12,4]: - 2 x[12,4] + w[12,4] <= 0
 VehicleRobotSynchronization1[12,5]: - 2 x[12,5] + w[12,5] <= 0
 VehicleRobotSynchronization1[12,6]: - 2 x[12,6] + w[12,6] <= 0
 VehicleRobotSynchronization1[12,7]: - 2 x[12,7] + w[12,7] <= 0
 VehicleRobotSynchronization1[12,8]: - 2 x[12,8] + w[12,8] <= 0
 VehicleRobotSynchronization1[12,9]: - 2 x[12,9] + w[12,9] <= 0
 VehicleRobotSynchronization1[12,10]: - 2 x[12,10] + w[12,10] <= 0
 VehicleRobotSynchronization1[12,11]: - 2 x[12,11] + w[12,11] <= 0
 VehicleRobotSynchronization1[12,13]: - 2 x[12,13] + w[12,13] <= 0
 SettingMode[2]: - w[2,7] + y[2,1] + 2 y[2,2] = 0
 SettingMode[3]: - w[3,8] + y[3,1] + 2 y[3,2] = 0
 SettingMode[4]: - w[4,9] + y[4,1] + 2 y[4,2] = 0
 SettingMode[5]: - w[5,10] + y[5,1] + 2 y[5,2] = 0
 SettingMode[6]: - w[6,11] + y[6,1] + 2 y[6,2] = 0
 SettingServiceTime[2]: - 350 y[2,1] - 175 y[2,2] + service_time[2] = 0
 SettingServiceTime[3]: - 410 y[3,1] - 205 y[3,2] + service_time[3] = 0
 SettingServiceTime[4]: - 450 y[4,1] - 225 y[4,2] + service_time[4] = 0
 SettingServiceTime[5]: - 200 y[5,1] - 100 y[5,2] + service_time[5] = 0
 SettingServiceTime[6]: - 230 y[6,1] - 115 y[6,2] + service_time[6] = 0
 AverageTimeRobots: - 8.891061343788897 w[0,2] - 12.87199784117568 w[0,3]
   - 6.044514958497615 w[0,4] - 10.52358408225586 w[0,5]
   - 10.63682886775472 w[0,6] - 8.891061343788897 w[0,7]
   - 12.87199784117568 w[0,8] - 6.044514958497615 w[0,9]
   - 10.52358408225586 w[0,10] - 10.63682886775472 w[0,11]
   - 8.891061343788897 w[1,2] - 12.87199784117568 w[1,3]
   - 6.044514958497615 w[1,4] - 10.52358408225586 w[1,5]
   - 10.63682886775472 w[1,6] - 8.891061343788897 w[1,7]
   - 12.87199784117568 w[1,8] - 6.044514958497615 w[1,9]
   - 10.52358408225586 w[1,10] - 10.63682886775472 w[1,11]
   - 8.891061343788897 w[2,1] - 19.62508224853268 w[2,3]
   - 11.08196078107399 w[2,4] - 18.89579826947694 w[2,5]
   - 17.29014733015933 w[2,6] - 19.62508224853268 w[2,8]
   - 11.08196078107399 w[2,9] - 18.89579826947694 w[2,10]
   - 17.29014733015933 w[2,11] - 8.891061343788897 w[2,12]
   - 8.891061343788897 w[2,13] - 12.87199784117568 w[3,1]
   - 19.62508224853268 w[3,2] - 8.555286283607849 w[3,4]
   - 14.96212422475996 w[3,5] - 19.04072139926705 w[3,6]
   - 19.62508224853268 w[3,7] - 8.555286283607849 w[3,9]
   - 14.96212422475996 w[3,10] - 19.04072139926705 w[3,11]
   - 12.87199784117568 w[3,12] - 12.87199784117568 w[3,13]
   - 6.044514958497615 w[4,1] - 11.08196078107399 w[4,2]
   - 8.555286283607849 w[4,3] - 14.02038553536676 w[4,5]
   - 15.83384171199285 w[4,6] - 11.08196078107399 w[4,7]
   - 8.555286283607849 w[4,8] - 14.02038553536676 w[4,10]
   - 15.83384171199285 w[4,11] - 6.044514958497615 w[4,12]
   - 6.044514958497615 w[4,13] - 10.52358408225586 w[5,1]
   - 18.89579826947694 w[5,2] - 14.96212422475996 w[5,3]
   - 14.02038553536676 w[5,4] - 5.297585469961459 w[5,6]
   - 18.89579826947694 w[5,7] - 14.96212422475996 w[5,8]
   - 14.02038553536676 w[5,9] - 5.297585469961459 w[5,11]
   - 10.52358408225586 w[5,12] - 10.52358408225586 w[5,13]
   - 10.63682886775472 w[6,1] - 17.29014733015933 w[6,2]
   - 19.04072139926705 w[6,3] - 15.83384171199285 w[6,4]
   - 5.297585469961459 w[6,5] - 17.29014733015933 w[6,7]
   - 19.04072139926705 w[6,8] - 15.83384171199285 w[6,9]
   - 5.297585469961459 w[6,10] - 10.63682886775472 w[6,12]
   - 10.63682886775472 w[6,13] - 8.891061343788897 w[7,1]
   - 19.62508224853268 w[7,3] - 11.08196078107399 w[7,4]
   - 18.89579826947694 w[7,5] - 17.29014733015933 w[7,6]
   - 19.62508224853268 w[7,8] - 11.08196078107399 w[7,9]
   - 18.89579826947694 w[7,10] - 17.29014733015933 w[7,11]
   - 8.891061343788897 w[7,12] - 8.891061343788897 w[7,13]
   - 12.87199784117568 w[8,1] - 19.62508224853268 w[8,2]
   - 8.555286283607849 w[8,4] - 14.96212422475996 w[8,5]
   - 19.04072139926705 w[8,6] - 19.62508224853268 w[8,7]
   - 8.555286283607849 w[8,9] - 14.96212422475996 w[8,10]
   - 19.04072139926705 w[8,11] - 12.87199784117568 w[8,12]
   - 12.87199784117568 w[8,13] - 6.044514958497615 w[9,1]
   - 11.08196078107399 w[9,2] - 8.555286283607849 w[9,3]
   - 14.02038553536676 w[9,5] - 15.83384171199285 w[9,6]
   - 11.08196078107399 w[9,7] - 8.555286283607849 w[9,8]
   - 14.02038553536676 w[9,10] - 15.83384171199285 w[9,11]
   - 6.044514958497615 w[9,12] - 6.044514958497615 w[9,13]
   - 10.52358408225586 w[10,1] - 18.89579826947694 w[10,2]
   - 14.96212422475996 w[10,3] - 14.02038553536676 w[10,4]
   - 5.297585469961459 w[10,6] - 18.89579826947694 w[10,7]
   - 14.96212422475996 w[10,8] - 14.02038553536676 w[10,9]
   - 5.297585469961459 w[10,11] - 10.52358408225586 w[10,12]
   - 10.52358408225586 w[10,13] - 10.63682886775472 w[11,1]
   - 17.29014733015933 w[11,2] - 19.04072139926705 w[11,3]
   - 15.83384171199285 w[11,4] - 5.297585469961459 w[11,5]
   - 17.29014733015933 w[11,7] - 19.04072139926705 w[11,8]
   - 15.83384171199285 w[11,9] - 5.297585469961459 w[11,10]
   - 10.63682886775472 w[11,12] - 10.63682886775472 w[11,13]
   - 8.891061343788897 w[12,2] - 12.87199784117568 w[12,3]
   - 6.044514958497615 w[12,4] - 10.52358408225586 w[12,5]
   - 10.63682886775472 w[12,6] - 8.891061343788897 w[12,7]
   - 12.87199784117568 w[12,8] - 6.044514958497615 w[12,9]
   - 10.52358408225586 w[12,10] - 10.63682886775472 w[12,11] - 70 y[2,1]
   - 140 y[2,2] - 82 y[3,1] - 164 y[3,2] - 90 y[4,1] - 180 y[4,2]
   - 40 y[5,1] - 80 y[5,2] - 46 y[6,1] - 92 y[6,2] + start_service[13]
   >= 0
 AverageTimeVehices: - 22.22765335947224 x[0,2] - 32.17999460293919 x[0,3]
   - 15.11128739624404 x[0,4] - 26.30896020563964 x[0,5]
   - 26.59207216938681 x[0,6] - 22.22765335947224 x[0,7]
   - 32.17999460293919 x[0,8] - 15.11128739624404 x[0,9]
   - 26.30896020563964 x[0,10] - 26.59207216938681 x[0,11]
   - 22.22765335947224 x[1,2] - 32.17999460293919 x[1,3]
   - 15.11128739624404 x[1,4] - 26.30896020563964 x[1,5]
   - 26.59207216938681 x[1,6] - 22.22765335947224 x[1,7]
   - 32.17999460293919 x[1,8] - 15.11128739624404 x[1,9]
   - 26.30896020563964 x[1,10] - 26.59207216938681 x[1,11]
   - 22.22765335947224 x[2,1] - 49.06270562133169 x[2,3]
   - 27.70490195268497 x[2,4] - 47.23949567369234 x[2,5]
   - 43.22536832539832 x[2,6] - 49.06270562133169 x[2,8]
   - 27.70490195268497 x[2,9] - 47.23949567369234 x[2,10]
   - 43.22536832539832 x[2,11] - 22.22765335947224 x[2,12]
   - 22.22765335947224 x[2,13] - 32.17999460293919 x[3,1]
   - 49.06270562133169 x[3,2] - 21.38821570901962 x[3,4]
   - 37.40531056189989 x[3,5] - 47.60180349816763 x[3,6]
   - 49.06270562133169 x[3,7] - 21.38821570901962 x[3,9]
   - 37.40531056189989 x[3,10] - 47.60180349816763 x[3,11]
   - 32.17999460293919 x[3,12] - 32.17999460293919 x[3,13]
   - 15.11128739624404 x[4,1] - 27.70490195268497 x[4,2]
   - 21.38821570901962 x[4,3] - 35.0509638384169 x[4,5]
   - 39.58460427998212 x[4,6] - 27.70490195268497 x[4,7]
   - 21.38821570901962 x[4,8] - 35.0509638384169 x[4,10]
   - 39.58460427998212 x[4,11] - 15.11128739624404 x[4,12]
   - 15.11128739624404 x[4,13] - 26.30896020563964 x[5,1]
   - 47.23949567369234 x[5,2] - 37.40531056189989 x[5,3]
   - 35.0509638384169 x[5,4] - 13.24396367490365 x[5,6]
   - 47.23949567369234 x[5,7] - 37.40531056189989 x[5,8]
   - 35.0509638384169 x[5,9] - 13.24396367490365 x[5,11]
   - 26.30896020563964 x[5,12] - 26.30896020563964 x[5,13]
   - 26.59207216938681 x[6,1] - 43.22536832539832 x[6,2]
   - 47.60180349816763 x[6,3] - 39.58460427998212 x[6,4]
   - 13.24396367490365 x[6,5] - 43.22536832539832 x[6,7]
   - 47.60180349816763 x[6,8] - 39.58460427998212 x[6,9]
   - 13.24396367490365 x[6,10] - 26.59207216938681 x[6,12]
   - 26.59207216938681 x[6,13] - 22.22765335947224 x[7,1]
   - 49.06270562133169 x[7,3] - 27.70490195268497 x[7,4]
   - 47.23949567369234 x[7,5] - 43.22536832539832 x[7,6]
   - 49.06270562133169 x[7,8] - 27.70490195268497 x[7,9]
   - 47.23949567369234 x[7,10] - 43.22536832539832 x[7,11]
   - 22.22765335947224 x[7,12] - 22.22765335947224 x[7,13]
   - 32.17999460293919 x[8,1] - 49.06270562133169 x[8,2]
   - 21.38821570901962 x[8,4] - 37.40531056189989 x[8,5]
   - 47.60180349816763 x[8,6] - 49.06270562133169 x[8,7]
   - 21.38821570901962 x[8,9] - 37.40531056189989 x[8,10]
   - 47.60180349816763 x[8,11] - 32.17999460293919 x[8,12]
   - 32.17999460293919 x[8,13] - 15.11128739624404 x[9,1]
   - 27.70490195268497 x[9,2] - 21.38821570901962 x[9,3]
   - 35.0509638384169 x[9,5] - 39.58460427998212 x[9,6]
   - 27.70490195268497 x[9,7] - 21.38821570901962 x[9,8]
   - 35.0509638384169 x[9,10] - 39.58460427998212 x[9,11]
   - 15.11128739624404 x[9,12] - 15.11128739624404 x[9,13]
   - 26.30896020563964 x[10,1] - 47.23949567369234 x[10,2]
   - 37.40531056189989 x[10,3] - 35.0509638384169 x[10,4]
   - 13.24396367490365 x[10,6] - 47.23949567369234 x[10,7]
   - 37.40531056189989 x[10,8] - 35.0509638384169 x[10,9]
   - 13.24396367490365 x[10,11] - 26.30896020563964 x[10,12]
   - 26.30896020563964 x[10,13] - 26.59207216938681 x[11,1]
   - 43.22536832539832 x[11,2] - 47.60180349816763 x[11,3]
   - 39.58460427998212 x[11,4] - 13.24396367490365 x[11,5]
   - 43.22536832539832 x[11,7] - 47.60180349816763 x[11,8]
   - 39.58460427998212 x[11,9] - 13.24396367490365 x[11,10]
   - 26.59207216938681 x[11,12] - 26.59207216938681 x[11,13]
   - 22.22765335947224 x[12,2] - 32.17999460293919 x[12,3]
   - 15.11128739624404 x[12,4] - 26.30896020563964 x[12,5]
   - 26.59207216938681 x[12,6] - 22.22765335947224 x[12,7]
   - 32.17999460293919 x[12,8] - 15.11128739624404 x[12,9]
   - 26.30896020563964 x[12,10] - 26.59207216938681 x[12,11]
   + start_service[13] >= 0
 LowerBoundRoutingVehicle[0]: - 88.91061343788895 x[0,2]
   - 128.7199784117568 x[0,3] - 60.44514958497615 x[0,4]
   - 105.2358408225586 x[0,5] - 106.3682886775472 x[0,6]
   - 88.91061343788895 x[0,7] - 128.7199784117568 x[0,8]
   - 60.44514958497615 x[0,9] - 105.2358408225586 x[0,10]
   - 106.3682886775472 x[0,11] - service_time[0] - start_service[0]
   + start_service[13] >= 0
 LowerBoundRoutingVehicle[1]: - 88.91061343788895 x[1,2]
   - 128.7199784117568 x[1,3] - 60.44514958497615 x[1,4]
   - 105.2358408225586 x[1,5] - 106.3682886775472 x[1,6]
   - 88.91061343788895 x[1,7] - 128.7199784117568 x[1,8]
   - 60.44514958497615 x[1,9] - 105.2358408225586 x[1,10]
   - 106.3682886775472 x[1,11] - service_time[1] - start_service[1]
   + start_service[13] >= 0
 LowerBoundRoutingVehicle[2]: - 44.45530671894448 x[2,1]
   - 44.45530671894448 x[2,2] - 162.4854004485418 x[2,3]
   - 85.632378697858 x[2,4] - 147.096911758664 x[2,5]
   - 139.6348809895703 x[2,6] - 44.45530671894448 x[2,7]
   - 162.4854004485418 x[2,8] - 85.632378697858 x[2,9]
   - 147.096911758664 x[2,10] - 139.6348809895703 x[2,11]
   - 44.45530671894448 x[2,12] - 44.45530671894448 x[2,13]
   - service_time[2] - start_service[2] + start_service[13] >= 0
 LowerBoundRoutingVehicle[3]: - 64.35998920587838 x[3,1]
   - 142.5807179616079 x[3,2] - 64.35998920587838 x[3,3]
   - 72.99900621052731 x[3,4] - 127.4285415350791 x[3,5]
   - 148.3877513351089 x[3,6] - 142.5807179616079 x[3,7]
   - 64.35998920587838 x[3,8] - 72.99900621052731 x[3,9]
   - 127.4285415350791 x[3,10] - 148.3877513351089 x[3,11]
   - 64.35998920587838 x[3,12] - 64.35998920587838 x[3,13]
   - service_time[3] - start_service[3] + start_service[13] >= 0
 LowerBoundRoutingVehicle[4]: - 30.22257479248807 x[4,1]
   - 99.86511062431441 x[4,2] - 107.1364206239176 x[4,3]
   - 30.22257479248807 x[4,4] - 122.7198480881131 x[4,5]
   - 132.3533528987379 x[4,6] - 99.86511062431441 x[4,7]
   - 107.1364206239176 x[4,8] - 30.22257479248807 x[4,9]
   - 122.7198480881131 x[4,10] - 132.3533528987379 x[4,11]
   - 30.22257479248807 x[4,12] - 30.22257479248807 x[4,13]
   - service_time[4] - start_service[4] + start_service[13] >= 0
 LowerBoundRoutingVehicle[5]: - 52.61792041127929 x[5,1]
   - 138.9342980663291 x[5,2] - 139.1706103296781 x[5,3]
   - 100.3245024693219 x[5,4] - 52.61792041127929 x[5,5]
   - 79.6720716885809 x[5,6] - 138.9342980663291 x[5,7]
   - 139.1706103296781 x[5,8] - 100.3245024693219 x[5,9]
   - 52.61792041127929 x[5,10] - 79.6720716885809 x[5,11]
   - 52.61792041127929 x[5,12] - 52.61792041127929 x[5,13]
   - service_time[5] - start_service[5] + start_service[13] >= 0
 LowerBoundRoutingVehicle[6]: - 53.18414433877361 x[6,1]
   - 130.9060433697411 x[6,2] - 159.5635962022137 x[6,3]
   - 109.3917833524523 x[6,4] - 79.10584776108658 x[6,5]
   - 53.18414433877361 x[6,6] - 130.9060433697411 x[6,7]
   - 159.5635962022137 x[6,8] - 109.3917833524523 x[6,9]
   - 79.10584776108658 x[6,10] - 53.18414433877361 x[6,11]
   - 53.18414433877361 x[6,12] - 53.18414433877361 x[6,13]
   - service_time[6] - start_service[6] + start_service[13] >= 0
 LowerBoundRoutingVehicle[7]: - 44.45530671894448 x[7,1]
   - 44.45530671894448 x[7,2] - 162.4854004485418 x[7,3]
   - 85.632378697858 x[7,4] - 147.096911758664 x[7,5]
   - 139.6348809895703 x[7,6] - 44.45530671894448 x[7,7]
   - 162.4854004485418 x[7,8] - 85.632378697858 x[7,9]
   - 147.096911758664 x[7,10] - 139.6348809895703 x[7,11]
   - 44.45530671894448 x[7,12] - 44.45530671894448 x[7,13]
   - service_time[7] - start_service[7] + start_service[13] >= 0
 LowerBoundRoutingVehicle[8]: - 64.35998920587838 x[8,1]
   - 142.5807179616079 x[8,2] - 64.35998920587838 x[8,3]
   - 72.99900621052731 x[8,4] - 127.4285415350791 x[8,5]
   - 148.3877513351089 x[8,6] - 142.5807179616079 x[8,7]
   - 64.35998920587838 x[8,8] - 72.99900621052731 x[8,9]
   - 127.4285415350791 x[8,10] - 148.3877513351089 x[8,11]
   - 64.35998920587838 x[8,12] - 64.35998920587838 x[8,13]
   - service_time[8] - start_service[8] + start_service[13] >= 0
 LowerBoundRoutingVehicle[9]: - 30.22257479248807 x[9,1]
   - 99.86511062431441 x[9,2] - 107.1364206239176 x[9,3]
   - 30.22257479248807 x[9,4] - 122.7198480881131 x[9,5]
   - 132.3533528987379 x[9,6] - 99.86511062431441 x[9,7]
   - 107.1364206239176 x[9,8] - 30.22257479248807 x[9,9]
   - 122.7198480881131 x[9,10] - 132.3533528987379 x[9,11]
   - 30.22257479248807 x[9,12] - 30.22257479248807 x[9,13]
   - service_time[9] - start_service[9] + start_service[13] >= 0
 LowerBoundRoutingVehicle[10]: - 52.61792041127929 x[10,1]
   - 138.9342980663291 x[10,2] - 139.1706103296781 x[10,3]
   - 100.3245024693219 x[10,4] - 52.61792041127929 x[10,5]
   - 79.6720716885809 x[10,6] - 138.9342980663291 x[10,7]
   - 139.1706103296781 x[10,8] - 100.3245024693219 x[10,9]
   - 52.61792041127929 x[10,10] - 79.6720716885809 x[10,11]
   - 52.61792041127929 x[10,12] - 52.61792041127929 x[10,13]
   - service_time[10] - start_service[10] + start_service[13] >= 0
 LowerBoundRoutingVehicle[11]: - 53.18414433877361 x[11,1]
   - 130.9060433697411 x[11,2] - 159.5635962022137 x[11,3]
   - 109.3917833524523 x[11,4] - 79.10584776108658 x[11,5]
   - 53.18414433877361 x[11,6] - 130.9060433697411 x[11,7]
   - 159.5635962022137 x[11,8] - 109.3917833524523 x[11,9]
   - 79.10584776108658 x[11,10] - 53.18414433877361 x[11,11]
   - 53.18414433877361 x[11,12] - 53.18414433877361 x[11,13]
   - service_time[11] - start_service[11] + start_service[13] >= 0
 LowerBoundRoutingVehicle[12]: - 88.91061343788895 x[12,2]
   - 128.7199784117568 x[12,3] - 60.44514958497615 x[12,4]
   - 105.2358408225586 x[12,5] - 106.3682886775472 x[12,6]
   - 88.91061343788895 x[12,7] - 128.7199784117568 x[12,8]
   - 60.44514958497615 x[12,9] - 105.2358408225586 x[12,10]
   - 106.3682886775472 x[12,11] - service_time[12] - start_service[12]
   + start_service[13] >= 0
 LowerBoundArrivalVehicle[1]: - 88.91061343788895 x[2,1]
   - 128.7199784117568 x[3,1] - 60.44514958497615 x[4,1]
   - 105.2358408225586 x[5,1] - 106.3682886775472 x[6,1]
   - 88.91061343788895 x[7,1] - 128.7199784117568 x[8,1]
   - 60.44514958497615 x[9,1] - 105.2358408225586 x[10,1]
   - 106.3682886775472 x[11,1] - start_service[0] + start_service[1] >= 0
 LowerBoundArrivalVehicle[2]: - 44.45530671894448 x[0,2]
   - 44.45530671894448 x[1,2] - 44.45530671894448 x[2,2]
   - 162.4854004485418 x[3,2] - 85.632378697858 x[4,2]
   - 147.096911758664 x[5,2] - 139.6348809895703 x[6,2]
   - 44.45530671894448 x[7,2] - 162.4854004485418 x[8,2]
   - 85.632378697858 x[9,2] - 147.096911758664 x[10,2]
   - 139.6348809895703 x[11,2] - 44.45530671894448 x[12,2]
   - start_service[0] + start_service[2] >= 0
 LowerBoundArrivalVehicle[3]: - 64.35998920587838 x[0,3]
   - 64.35998920587838 x[1,3] - 142.5807179616079 x[2,3]
   - 64.35998920587838 x[3,3] - 72.99900621052731 x[4,3]
   - 127.4285415350791 x[5,3] - 148.3877513351089 x[6,3]
   - 142.5807179616079 x[7,3] - 64.35998920587838 x[8,3]
   - 72.99900621052731 x[9,3] - 127.4285415350791 x[10,3]
   - 148.3877513351089 x[11,3] - 64.35998920587838 x[12,3]
   - start_service[0] + start_service[3] >= 0
 LowerBoundArrivalVehicle[4]: - 30.22257479248807 x[0,4]
   - 30.22257479248807 x[1,4] - 99.86511062431441 x[2,4]
   - 107.1364206239176 x[3,4] - 30.22257479248807 x[4,4]
   - 122.7198480881131 x[5,4] - 132.3533528987379 x[6,4]
   - 99.86511062431441 x[7,4] - 107.1364206239176 x[8,4]
   - 30.22257479248807 x[9,4] - 122.7198480881131 x[10,4]
   - 132.3533528987379 x[11,4] - 30.22257479248807 x[12,4]
   - start_service[0] + start_service[4] >= 0
 LowerBoundArrivalVehicle[5]: - 52.61792041127929 x[0,5]
   - 52.61792041127929 x[1,5] - 138.9342980663291 x[2,5]
   - 139.1706103296781 x[3,5] - 100.3245024693219 x[4,5]
   - 52.61792041127929 x[5,5] - 79.6720716885809 x[6,5]
   - 138.9342980663291 x[7,5] - 139.1706103296781 x[8,5]
   - 100.3245024693219 x[9,5] - 52.61792041127929 x[10,5]
   - 79.6720716885809 x[11,5] - 52.61792041127929 x[12,5]
   - start_service[0] + start_service[5] >= 0
 LowerBoundArrivalVehicle[6]: - 53.18414433877361 x[0,6]
   - 53.18414433877361 x[1,6] - 130.9060433697411 x[2,6]
   - 159.5635962022137 x[3,6] - 109.3917833524523 x[4,6]
   - 79.10584776108658 x[5,6] - 53.18414433877361 x[6,6]
   - 130.9060433697411 x[7,6] - 159.5635962022137 x[8,6]
   - 109.3917833524523 x[9,6] - 79.10584776108658 x[10,6]
   - 53.18414433877361 x[11,6] - 53.18414433877361 x[12,6]
   - start_service[0] + start_service[6] >= 0
 LowerBoundArrivalVehicle[7]: - 44.45530671894448 x[0,7]
   - 44.45530671894448 x[1,7] - 44.45530671894448 x[2,7]
   - 162.4854004485418 x[3,7] - 85.632378697858 x[4,7]
   - 147.096911758664 x[5,7] - 139.6348809895703 x[6,7]
   - 44.45530671894448 x[7,7] - 162.4854004485418 x[8,7]
   - 85.632378697858 x[9,7] - 147.096911758664 x[10,7]
   - 139.6348809895703 x[11,7] - 44.45530671894448 x[12,7]
   - start_service[0] + start_service[7] >= 0
 LowerBoundArrivalVehicle[8]: - 64.35998920587838 x[0,8]
   - 64.35998920587838 x[1,8] - 142.5807179616079 x[2,8]
   - 64.35998920587838 x[3,8] - 72.99900621052731 x[4,8]
   - 127.4285415350791 x[5,8] - 148.3877513351089 x[6,8]
   - 142.5807179616079 x[7,8] - 64.35998920587838 x[8,8]
   - 72.99900621052731 x[9,8] - 127.4285415350791 x[10,8]
   - 148.3877513351089 x[11,8] - 64.35998920587838 x[12,8]
   - start_service[0] + start_service[8] >= 0
 LowerBoundArrivalVehicle[9]: - 30.22257479248807 x[0,9]
   - 30.22257479248807 x[1,9] - 99.86511062431441 x[2,9]
   - 107.1364206239176 x[3,9] - 30.22257479248807 x[4,9]
   - 122.7198480881131 x[5,9] - 132.3533528987379 x[6,9]
   - 99.86511062431441 x[7,9] - 107.1364206239176 x[8,9]
   - 30.22257479248807 x[9,9] - 122.7198480881131 x[10,9]
   - 132.3533528987379 x[11,9] - 30.22257479248807 x[12,9]
   - start_service[0] + start_service[9] >= 0
 LowerBoundArrivalVehicle[10]: - 52.61792041127929 x[0,10]
   - 52.61792041127929 x[1,10] - 138.9342980663291 x[2,10]
   - 139.1706103296781 x[3,10] - 100.3245024693219 x[4,10]
   - 52.61792041127929 x[5,10] - 79.6720716885809 x[6,10]
   - 138.9342980663291 x[7,10] - 139.1706103296781 x[8,10]
   - 100.3245024693219 x[9,10] - 52.61792041127929 x[10,10]
   - 79.6720716885809 x[11,10] - 52.61792041127929 x[12,10]
   - start_service[0] + start_service[10] >= 0
 LowerBoundArrivalVehicle[11]: - 53.18414433877361 x[0,11]
   - 53.18414433877361 x[1,11] - 130.9060433697411 x[2,11]
   - 159.5635962022137 x[3,11] - 109.3917833524523 x[4,11]
   - 79.10584776108658 x[5,11] - 53.18414433877361 x[6,11]
   - 130.9060433697411 x[7,11] - 159.5635962022137 x[8,11]
   - 109.3917833524523 x[9,11] - 79.10584776108658 x[10,11]
   - 53.18414433877361 x[11,11] - 53.18414433877361 x[12,11]
   - start_service[0] + start_service[11] >= 0
 LowerBoundArrivalVehicle[12]: - 88.91061343788895 x[2,12]
   - 128.7199784117568 x[3,12] - 60.44514958497615 x[4,12]
   - 105.2358408225586 x[5,12] - 106.3682886775472 x[6,12]
   - 88.91061343788895 x[7,12] - 128.7199784117568 x[8,12]
   - 60.44514958497615 x[9,12] - 105.2358408225586 x[10,12]
   - 106.3682886775472 x[11,12] - start_service[0] + start_service[12]
   >= 0
 LowerBoundArrivalVehicle[13]: - 88.91061343788895 x[2,13]
   - 128.7199784117568 x[3,13] - 60.44514958497615 x[4,13]
   - 105.2358408225586 x[5,13] - 106.3682886775472 x[6,13]
   - 88.91061343788895 x[7,13] - 128.7199784117568 x[8,13]
   - 60.44514958497615 x[9,13] - 105.2358408225586 x[10,13]
   - 106.3682886775472 x[11,13] - start_service[0] + start_service[13]
   >= 0
 VisitArrivalNodesNotEmpty[2]: w[0,2] + w[1,2] + w[2,2] + w[3,2] + w[4,2]
   + w[5,2] + w[6,2] + w[7,2] + w[8,2] + w[9,2] + w[10,2] + w[11,2]
   + w[12,2] + w[13,2] >= 1
 VisitArrivalNodesNotEmpty[3]: w[0,3] + w[1,3] + w[2,3] + w[3,3] + w[4,3]
   + w[5,3] + w[6,3] + w[7,3] + w[8,3] + w[9,3] + w[10,3] + w[11,3]
   + w[12,3] + w[13,3] >= 1
 VisitArrivalNodesNotEmpty[4]: w[0,4] + w[1,4] + w[2,4] + w[3,4] + w[4,4]
   + w[5,4] + w[6,4] + w[7,4] + w[8,4] + w[9,4] + w[10,4] + w[11,4]
   + w[12,4] + w[13,4] >= 1
 VisitArrivalNodesNotEmpty[5]: w[0,5] + w[1,5] + w[2,5] + w[3,5] + w[4,5]
   + w[5,5] + w[6,5] + w[7,5] + w[8,5] + w[9,5] + w[10,5] + w[11,5]
   + w[12,5] + w[13,5] >= 1
 VisitArrivalNodesNotEmpty[6]: w[0,6] + w[1,6] + w[2,6] + w[3,6] + w[4,6]
   + w[5,6] + w[6,6] + w[7,6] + w[8,6] + w[9,6] + w[10,6] + w[11,6]
   + w[12,6] + w[13,6] >= 1
 LeaveArrivalNodesNotFullyLoaded[2]: w[2,0] + w[2,1] + w[2,3] + w[2,4]
   + w[2,5] + w[2,6] + w[2,8] + w[2,9] + w[2,10] + w[2,11] + w[2,12]
   + w[2,13] <= 1
 LeaveArrivalNodesNotFullyLoaded[3]: w[3,0] + w[3,1] + w[3,2] + w[3,4]
   + w[3,5] + w[3,6] + w[3,7] + w[3,9] + w[3,10] + w[3,11] + w[3,12]
   + w[3,13] <= 1
 LeaveArrivalNodesNotFullyLoaded[4]: w[4,0] + w[4,1] + w[4,2] + w[4,3]
   + w[4,5] + w[4,6] + w[4,7] + w[4,8] + w[4,10] + w[4,11] + w[4,12]
   + w[4,13] <= 1
 LeaveArrivalNodesNotFullyLoaded[5]: w[5,0] + w[5,1] + w[5,2] + w[5,3]
   + w[5,4] + w[5,6] + w[5,7] + w[5,8] + w[5,9] + w[5,11] + w[5,12]
   + w[5,13] <= 1
 LeaveArrivalNodesNotFullyLoaded[6]: w[6,0] + w[6,1] + w[6,2] + w[6,3]
   + w[6,4] + w[6,5] + w[6,7] + w[6,8] + w[6,9] + w[6,10] + w[6,12]
   + w[6,13] <= 1
 VisitDepartureNodesNotFullyLoaded[7]: w[0,7] + w[1,7] + w[3,7] + w[4,7]
   + w[5,7] + w[6,7] + w[8,7] + w[9,7] + w[10,7] + w[11,7] + w[12,7]
   + w[13,7] <= 1
 VisitDepartureNodesNotFullyLoaded[8]: w[0,8] + w[1,8] + w[2,8] + w[4,8]
   + w[5,8] + w[6,8] + w[7,8] + w[9,8] + w[10,8] + w[11,8] + w[12,8]
   + w[13,8] <= 1
 VisitDepartureNodesNotFullyLoaded[9]: w[0,9] + w[1,9] + w[2,9] + w[3,9]
   + w[5,9] + w[6,9] + w[7,9] + w[8,9] + w[10,9] + w[11,9] + w[12,9]
   + w[13,9] <= 1
 VisitDepartureNodesNotFullyLoaded[10]: w[0,10] + w[1,10] + w[2,10]
   + w[3,10] + w[4,10] + w[6,10] + w[7,10] + w[8,10] + w[9,10] + w[11,10]
   + w[12,10] + w[13,10] <= 1
 VisitDepartureNodesNotFullyLoaded[11]: w[0,11] + w[1,11] + w[2,11]
   + w[3,11] + w[4,11] + w[5,11] + w[7,11] + w[8,11] + w[9,11] + w[10,11]
   + w[12,11] + w[13,11] <= 1
 LeaveDepartureNodesNotEmpty[7]: w[7,0] + w[7,1] + w[7,2] + w[7,3] + w[7,4]
   + w[7,5] + w[7,6] + w[7,7] + w[7,8] + w[7,9] + w[7,10] + w[7,11]
   + w[7,12] + w[7,13] >= 1
 LeaveDepartureNodesNotEmpty[8]: w[8,0] + w[8,1] + w[8,2] + w[8,3] + w[8,4]
   + w[8,5] + w[8,6] + w[8,7] + w[8,8] + w[8,9] + w[8,10] + w[8,11]
   + w[8,12] + w[8,13] >= 1
 LeaveDepartureNodesNotEmpty[9]: w[9,0] + w[9,1] + w[9,2] + w[9,3] + w[9,4]
   + w[9,5] + w[9,6] + w[9,7] + w[9,8] + w[9,9] + w[9,10] + w[9,11]
   + w[9,12] + w[9,13] >= 1
 LeaveDepartureNodesNotEmpty[10]: w[10,0] + w[10,1] + w[10,2] + w[10,3]
   + w[10,4] + w[10,5] + w[10,6] + w[10,7] + w[10,8] + w[10,9] + w[10,10]
   + w[10,11] + w[10,12] + w[10,13] >= 1
 LeaveDepartureNodesNotEmpty[11]: w[11,0] + w[11,1] + w[11,2] + w[11,3]
   + w[11,4] + w[11,5] + w[11,6] + w[11,7] + w[11,8] + w[11,9] + w[11,10]
   + w[11,11] + w[11,12] + w[11,13] >= 1
Bounds
 x[0,0] = 0
 x[1,0] = 0
 x[1,1] = 0
 x[2,0] = 0
 x[2,2] = 0
 x[3,0] = 0
 x[3,3] = 0
 x[4,0] = 0
 x[4,4] = 0
 x[5,0] = 0
 x[5,5] = 0
 x[6,0] = 0
 x[6,6] = 0
 x[7,0] = 0
 x[7,2] = 0
 x[7,7] = 0
 x[8,0] = 0
 x[8,3] = 0
 x[8,8] = 0
 x[9,0] = 0
 x[9,4] = 0
 x[9,9] = 0
 x[10,0] = 0
 x[10,5] = 0
 x[10,10] = 0
 x[11,0] = 0
 x[11,6] = 0
 x[11,11] = 0
 x[12,0] = 0
 x[12,12] = 0
 x[13,0] = 0
 x[13,1] = 0
 x[13,2] = 0
 x[13,3] = 0
 x[13,4] = 0
 x[13,5] = 0
 x[13,6] = 0
 x[13,7] = 0
 x[13,8] = 0
 x[13,9] = 0
 x[13,10] = 0
 x[13,11] = 0
 x[13,12] = 0
 x[13,13] = 0
 w[0,0] = -0
 w[0,1] = -0
 w[0,2] <= 2
 w[0,3] <= 2
 w[0,4] <= 2
 w[0,5] <= 2
 w[0,6] <= 2
 w[0,7] <= 2
 w[0,8] <= 2
 w[0,9] <= 2
 w[0,10] <= 2
 w[0,11] <= 2
 w[0,12] <= 2
 w[0,13] <= 2
 w[1,0] = -0
 w[1,1] = -0
 w[1,2] <= 2
 w[1,3] <= 2
 w[1,4] <= 2
 w[1,5] <= 2
 w[1,6] <= 2
 w[1,7] <= 2
 w[1,8] <= 2
 w[1,9] <= 2
 w[1,10] <= 2
 w[1,11] <= 2
 w[1,12] <= 2
 w[1,13] <= 2
 w[2,0] = -0
 w[2,1] = -0
 w[2,2] = -0
 w[2,3] <= 2
 w[2,4] <= 2
 w[2,5] <= 2
 w[2,6] <= 2
 w[2,7] <= 2
 w[2,8] <= 2
 w[2,9] <= 2
 w[2,10] <= 2
 w[2,11] <= 2
 w[2,12] <= 2
 w[2,13] <= 2
 w[3,0] = -0
 w[3,1] = -0
 w[3,2] <= 2
 w[3,3] = -0
 w[3,4] <= 2
 w[3,5] <= 2
 w[3,6] <= 2
 w[3,7] <= 2
 w[3,8] <= 2
 w[3,9] <= 2
 w[3,10] <= 2
 w[3,11] <= 2
 w[3,12] <= 2
 w[3,13] <= 2
 w[4,0] = -0
 w[4,1] = -0
 w[4,2] <= 2
 w[4,3] <= 2
 w[4,4] = -0
 w[4,5] <= 2
 w[4,6] <= 2
 w[4,7] <= 2
 w[4,8] <= 2
 w[4,9] <= 2
 w[4,10] <= 2
 w[4,11] <= 2
 w[4,12] <= 2
 w[4,13] <= 2
 w[5,0] = -0
 w[5,1] = -0
 w[5,2] <= 2
 w[5,3] <= 2
 w[5,4] <= 2
 w[5,5] = -0
 w[5,6] <= 2
 w[5,7] <= 2
 w[5,8] <= 2
 w[5,9] <= 2
 w[5,10] <= 2
 w[5,11] <= 2
 w[5,12] <= 2
 w[5,13] <= 2
 w[6,0] = -0
 w[6,1] = -0
 w[6,2] <= 2
 w[6,3] <= 2
 w[6,4] <= 2
 w[6,5] <= 2
 w[6,6] = -0
 w[6,7] <= 2
 w[6,8] <= 2
 w[6,9] <= 2
 w[6,10] <= 2
 w[6,11] <= 2
 w[6,12] <= 2
 w[6,13] <= 2
 w[7,0] = -0
 w[7,1] = -0
 w[7,2] = -0
 w[7,3] <= 2
 w[7,4] <= 2
 w[7,5] <= 2
 w[7,6] <= 2
 w[7,7] = -0
 w[7,8] <= 2
 w[7,9] <= 2
 w[7,10] <= 2
 w[7,11] <= 2
 w[7,12] <= 2
 w[7,13] <= 2
 w[8,0] = -0
 w[8,1] = -0
 w[8,2] <= 2
 w[8,3] = -0
 w[8,4] <= 2
 w[8,5] <= 2
 w[8,6] <= 2
 w[8,7] <= 2
 w[8,8] = -0
 w[8,9] <= 2
 w[8,10] <= 2
 w[8,11] <= 2
 w[8,12] <= 2
 w[8,13] <= 2
 w[9,0] = -0
 w[9,1] = -0
 w[9,2] <= 2
 w[9,3] <= 2
 w[9,4] = -0
 w[9,5] <= 2
 w[9,6] <= 2
 w[9,7] <= 2
 w[9,8] <= 2
 w[9,9] = -0
 w[9,10] <= 2
 w[9,11] <= 2
 w[9,12] <= 2
 w[9,13] <= 2
 w[10,0] = -0
 w[10,1] = -0
 w[10,2] <= 2
 w[10,3] <= 2
 w[10,4] <= 2
 w[10,5] = -0
 w[10,6] <= 2
 w[10,7] <= 2
 w[10,8] <= 2
 w[10,9] <= 2
 w[10,10] = -0
 w[10,11] <= 2
 w[10,12] <= 2
 w[10,13] <= 2
 w[11,0] = -0
 w[11,1] = -0
 w[11,2] <= 2
 w[11,3] <= 2
 w[11,4] <= 2
 w[11,5] <= 2
 w[11,6] = -0
 w[11,7] <= 2
 w[11,8] <= 2
 w[11,9] <= 2
 w[11,10] <= 2
 w[11,11] = -0
 w[11,12] <= 2
 w[11,13] <= 2
 w[12,0] = -0
 w[12,1] = -0
 w[12,2] = -0
 w[12,3] = -0
 w[12,4] = -0
 w[12,5] = -0
 w[12,6] = -0
 w[12,7] = -0
 w[12,8] = -0
 w[12,9] = -0
 w[12,10] = -0
 w[12,11] = -0
 w[12,12] = -0
 w[12,13] = -0
 w[13,0] = -0
 w[13,1] = -0
 w[13,2] = -0
 w[13,3] = -0
 w[13,4] = -0
 w[13,5] = -0
 w[13,6] = -0
 w[13,7] = -0
 w[13,8] = -0
 w[13,9] = -0
 w[13,10] = -0
 w[13,11] = -0
 w[13,12] = -0
 w[13,13] = -0
 v[0,0] = 0
 v[0,1] = 0
 v[1,0] = 0
 v[1,1] = 0
 v[2,0] = 0
 v[2,1] = 0
 v[2,2] = 0
 v[2,7] = 1
 v[3,0] = 0
 v[3,1] = 0
 v[3,3] = 0
 v[3,8] = 1
 v[4,0] = 0
 v[4,1] = 0
 v[4,4] = 0
 v[4,9] = 1
 v[5,0] = 0
 v[5,1] = 0
 v[5,5] = 0
 v[5,10] = 1
 v[6,0] = 0
 v[6,1] = 0
 v[6,6] = 0
 v[6,11] = 1
 v[7,0] = 0
 v[7,1] = 0
 v[7,2] = 0
 v[7,7] = 0
 v[8,0] = 0
 v[8,1] = 0
 v[8,3] = 0
 v[8,8] = 0
 v[9,0] = 0
 v[9,1] = 0
 v[9,4] = 0
 v[9,9] = 0
 v[10,0] = 0
 v[10,1] = 0
 v[10,5] = 0
 v[10,10] = 0
 v[11,0] = 0
 v[11,1] = 0
 v[11,6] = 0
 v[11,11] = 0
 v[12,0] = 0
 v[12,1] = 0
 v[12,2] = 0
 v[12,3] = 0
 v[12,4] = 0
 v[12,5] = 0
 v[12,6] = 0
 v[12,7] = 0
 v[12,8] = 0
 v[12,9] = 0
 v[12,10] = 0
 v[12,11] = 0
 v[12,12] = 0
 v[12,13] = 0
 v[13,0] = 0
 v[13,1] = 0
 v[13,2] = 0
 v[13,3] = 0
 v[13,4] = 0
 v[13,5] = 0
 v[13,6] = 0
 v[13,7] = 0
 v[13,8] = 0
 v[13,9] = 0
 v[13,10] = 0
 v[13,11] = 0
 v[13,12] = 0
 v[13,13] = 0
 service_time[0] = 0
 service_time[1] = 0
 service_time[7] = 0
 service_time[8] = 0
 service_time[9] = 0
 service_time[10] = 0
 service_time[11] = 0
 service_time[12] = 0
 service_time[13] = 0
 start_service[0] = 0
 start_service[1] <= 10000
 start_service[2] <= 10000
 start_service[3] <= 10000
 start_service[4] <= 10000
 start_service[5] <= 10000
 start_service[6] <= 10000
 start_service[7] <= 10000
 start_service[8] <= 10000
 start_service[9] <= 10000
 start_service[10] <= 10000
 start_service[11] <= 10000
 start_service[12] <= 10000
 start_service[13] <= 10000
Binaries
 x[0,0] x[0,1] x[0,2] x[0,3] x[0,4] x[0,5] x[0,6] x[0,7] x[0,8] x[0,9]
 x[0,10] x[0,11] x[0,12] x[0,13] x[1,0] x[1,1] x[1,2] x[1,3] x[1,4] x[1,5]
 x[1,6] x[1,7] x[1,8] x[1,9] x[1,10] x[1,11] x[1,12] x[1,13] x[2,0] x[2,1]
 x[2,2] x[2,3] x[2,4] x[2,5] x[2,6] x[2,7] x[2,8] x[2,9] x[2,10] x[2,11]
 x[2,12] x[2,13] x[3,0] x[3,1] x[3,2] x[3,3] x[3,4] x[3,5] x[3,6] x[3,7]
 x[3,8] x[3,9] x[3,10] x[3,11] x[3,12] x[3,13] x[4,0] x[4,1] x[4,2] x[4,3]
 x[4,4] x[4,5] x[4,6] x[4,7] x[4,8] x[4,9] x[4,10] x[4,11] x[4,12] x[4,13]
 x[5,0] x[5,1] x[5,2] x[5,3] x[5,4] x[5,5] x[5,6] x[5,7] x[5,8] x[5,9]
 x[5,10] x[5,11] x[5,12] x[5,13] x[6,0] x[6,1] x[6,2] x[6,3] x[6,4] x[6,5]
 x[6,6] x[6,7] x[6,8] x[6,9] x[6,10] x[6,11] x[6,12] x[6,13] x[7,0] x[7,1]
 x[7,2] x[7,3] x[7,4] x[7,5] x[7,6] x[7,7] x[7,8] x[7,9] x[7,10] x[7,11]
 x[7,12] x[7,13] x[8,0] x[8,1] x[8,2] x[8,3] x[8,4] x[8,5] x[8,6] x[8,7]
 x[8,8] x[8,9] x[8,10] x[8,11] x[8,12] x[8,13] x[9,0] x[9,1] x[9,2] x[9,3]
 x[9,4] x[9,5] x[9,6] x[9,7] x[9,8] x[9,9] x[9,10] x[9,11] x[9,12] x[9,13]
 x[10,0] x[10,1] x[10,2] x[10,3] x[10,4] x[10,5] x[10,6] x[10,7] x[10,8]
 x[10,9] x[10,10] x[10,11] x[10,12] x[10,13] x[11,0] x[11,1] x[11,2]
 x[11,3] x[11,4] x[11,5] x[11,6] x[11,7] x[11,8] x[11,9] x[11,10] x[11,11]
 x[11,12] x[11,13] x[12,0] x[12,1] x[12,2] x[12,3] x[12,4] x[12,5] x[12,6]
 x[12,7] x[12,8] x[12,9] x[12,10] x[12,11] x[12,12] x[12,13] x[13,0]
 x[13,1] x[13,2] x[13,3] x[13,4] x[13,5] x[13,6] x[13,7] x[13,8] x[13,9]
 x[13,10] x[13,11] x[13,12] x[13,13] v[0,0] v[0,1] v[0,2] v[0,3] v[0,4]
 v[0,5] v[0,6] v[0,7] v[0,8] v[0,9] v[0,10] v[0,11] v[0,12] v[0,13] v[1,0]
 v[1,1] v[1,2] v[1,3] v[1,4] v[1,5] v[1,6] v[1,7] v[1,8] v[1,9] v[1,10]
 v[1,11] v[1,12] v[1,13] v[2,0] v[2,1] v[2,2] v[2,3] v[2,4] v[2,5] v[2,6]
 v[2,7] v[2,8] v[2,9] v[2,10] v[2,11] v[2,12] v[2,13] v[3,0] v[3,1] v[3,2]
 v[3,3] v[3,4] v[3,5] v[3,6] v[3,7] v[3,8] v[3,9] v[3,10] v[3,11] v[3,12]
 v[3,13] v[4,0] v[4,1] v[4,2] v[4,3] v[4,4] v[4,5] v[4,6] v[4,7] v[4,8]
 v[4,9] v[4,10] v[4,11] v[4,12] v[4,13] v[5,0] v[5,1] v[5,2] v[5,3] v[5,4]
 v[5,5] v[5,6] v[5,7] v[5,8] v[5,9] v[5,10] v[5,11] v[5,12] v[5,13] v[6,0]
 v[6,1] v[6,2] v[6,3] v[6,4] v[6,5] v[6,6] v[6,7] v[6,8] v[6,9] v[6,10]
 v[6,11] v[6,12] v[6,13] v[7,0] v[7,1] v[7,2] v[7,3] v[7,4] v[7,5] v[7,6]
 v[7,7] v[7,8] v[7,9] v[7,10] v[7,11] v[7,12] v[7,13] v[8,0] v[8,1] v[8,2]
 v[8,3] v[8,4] v[8,5] v[8,6] v[8,7] v[8,8] v[8,9] v[8,10] v[8,11] v[8,12]
 v[8,13] v[9,0] v[9,1] v[9,2] v[9,3] v[9,4] v[9,5] v[9,6] v[9,7] v[9,8]
 v[9,9] v[9,10] v[9,11] v[9,12] v[9,13] v[10,0] v[10,1] v[10,2] v[10,3]
 v[10,4] v[10,5] v[10,6] v[10,7] v[10,8] v[10,9] v[10,10] v[10,11] v[10,12]
 v[10,13] v[11,0] v[11,1] v[11,2] v[11,3] v[11,4] v[11,5] v[11,6] v[11,7]
 v[11,8] v[11,9] v[11,10] v[11,11] v[11,12] v[11,13] v[12,0] v[12,1]
 v[12,2] v[12,3] v[12,4] v[12,5] v[12,6] v[12,7] v[12,8] v[12,9] v[12,10]
 v[12,11] v[12,12] v[12,13] v[13,0] v[13,1] v[13,2] v[13,3] v[13,4] v[13,5]
 v[13,6] v[13,7] v[13,8] v[13,9] v[13,10] v[13,11] v[13,12] v[13,13] y[2,1]
 y[2,2] y[3,1] y[3,2] y[4,1] y[4,2] y[5,1] y[5,2] y[6,1] y[6,2]
Generals
 w[0,0] w[0,1] w[0,2] w[0,3] w[0,4] w[0,5] w[0,6] w[0,7] w[0,8] w[0,9]
 w[0,10] w[0,11] w[0,12] w[0,13] w[1,0] w[1,1] w[1,2] w[1,3] w[1,4] w[1,5]
 w[1,6] w[1,7] w[1,8] w[1,9] w[1,10] w[1,11] w[1,12] w[1,13] w[2,0] w[2,1]
 w[2,2] w[2,3] w[2,4] w[2,5] w[2,6] w[2,7] w[2,8] w[2,9] w[2,10] w[2,11]
 w[2,12] w[2,13] w[3,0] w[3,1] w[3,2] w[3,3] w[3,4] w[3,5] w[3,6] w[3,7]
 w[3,8] w[3,9] w[3,10] w[3,11] w[3,12] w[3,13] w[4,0] w[4,1] w[4,2] w[4,3]
 w[4,4] w[4,5] w[4,6] w[4,7] w[4,8] w[4,9] w[4,10] w[4,11] w[4,12] w[4,13]
 w[5,0] w[5,1] w[5,2] w[5,3] w[5,4] w[5,5] w[5,6] w[5,7] w[5,8] w[5,9]
 w[5,10] w[5,11] w[5,12] w[5,13] w[6,0] w[6,1] w[6,2] w[6,3] w[6,4] w[6,5]
 w[6,6] w[6,7] w[6,8] w[6,9] w[6,10] w[6,11] w[6,12] w[6,13] w[7,0] w[7,1]
 w[7,2] w[7,3] w[7,4] w[7,5] w[7,6] w[7,7] w[7,8] w[7,9] w[7,10] w[7,11]
 w[7,12] w[7,13] w[8,0] w[8,1] w[8,2] w[8,3] w[8,4] w[8,5] w[8,6] w[8,7]
 w[8,8] w[8,9] w[8,10] w[8,11] w[8,12] w[8,13] w[9,0] w[9,1] w[9,2] w[9,3]
 w[9,4] w[9,5] w[9,6] w[9,7] w[9,8] w[9,9] w[9,10] w[9,11] w[9,12] w[9,13]
 w[10,0] w[10,1] w[10,2] w[10,3] w[10,4] w[10,5] w[10,6] w[10,7] w[10,8]
 w[10,9] w[10,10] w[10,11] w[10,12] w[10,13] w[11,0] w[11,1] w[11,2]
 w[11,3] w[11,4] w[11,5] w[11,6] w[11,7] w[11,8] w[11,9] w[11,10] w[11,11]
 w[11,12] w[11,13] w[12,0] w[12,1] w[12,2] w[12,3] w[12,4] w[12,5] w[12,6]
 w[12,7] w[12,8] w[12,9] w[12,10] w[12,11] w[12,12] w[12,13] w[13,0]
 w[13,1] w[13,2] w[13,3] w[13,4] w[13,5] w[13,6] w[13,7] w[13,8] w[13,9]
 w[13,10] w[13,11] w[13,12] w[13,13]
End
