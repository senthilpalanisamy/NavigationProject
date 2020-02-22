## Experimental results ##

| EXPERIMENT | FV     | AV     | ET    | EX     | EY     | OT   | OX    | 0Y   | FT   | FX    | FY    | GT    | GX    | GY    | DT    | DX    | DY|
|:---:        |:---:  | :---:  | :---: |:--:   | :---:  | :---:| :---: |:---: | :---: | :---: |:---: |:---:  |:---:  |:---:  |:---:  |:---:  |:---:|
| CCW         | 1     | 2.84   |  0    |    0   |   0    | 1.53 | 0.03  | 0.04 | 1.776 | 0     | 0     |   1.53| 0     |   0   | 0.0765| 0     | 0     |
| CCW         |1      |	2.84   |	0    |	0     |	0      |	1.53|	0.03  |	0.04 |	1.776|	0	   |0	     |1.53	 | 0	   |0	     |0.0765 |0	     |0      |
| CW          |1      |	2.84   |	0    |0	      |0	     |-1.65 |	-0.01 |	0	   |-0.8   |	0	   |0	     |-1.134 |	0    |	0	   |-0.0567|0	     |0      |
| CCW         |0.3    |	0.852  |	0	   |0       |	0      |	1.44|	0.023	|-0.02 |	1.514|	0    |	0    |	1.5  |	0	   |0	     |0.075  |	0    |	0    |
| CW          |0.3    |	0.852  |	0    |	0     |	0	     |-1.524|	-0.01 |	0	   |-3.103 |	0	   |0	     |1.553  |	0    |	0    |0.07765|	0    |	0    |
| FWD         |1	    |2.84    |	0	   |2	      |0	     |0.221 |	1.81  |	0.2	 |0	     |2.003  |	0	   |1.81	 |0.15   |	0.08 |0.0905 |0.0075 |	0.004|
| BWD         |1	    |2.84    |	0    |	-2    |0	     |0.05  |	-1.949|	0.1  |	0	   |-2.004 |	0	   |0.08   |	-1.88| 0.165 |0.004	 |-0.094 |0.00825|
| FWD         |0.3    |	0.852  |	0	   |2	      |0	     |0.02  |	2.004 |	0	   |0      |2.004  |	0	   |0.04   |	1.98 |	0.1  |	0.004|0.002	 |-0.01  |
| BWD         |0.3    | 0.852  |	0	   |-2	    |0	     |-0.03 |	-2.029|	0.03 |	0	   |-2.015 |	0	   |0	     |-2     |	0.08 |	0	   |0	     |-0.008 |



## Why is odometry superior than feed forward estimation?##

Yes, using odometers improves the pose estimates as opposed to using a feedforward model.
Some of the factors that odometers account for, that feedforward models don't account for are
1. Dynamics / Higher order control. Our feedforward model is a simple velocity based control,which
  assumes that velocities are achieved instantaneously, the moment commands are applied but
  as we know, this is far away from the truth. Therefore, odometry based updates are more accurate
2. Unmodelled system characteristics: There are lot of unmodeled effects like fricition between
   the wheel and the floor, motors may not exactly replicate the desired commands etc..
   There are captured by the odomery. There are things that are even beyond the grasp of
   odometry like wheel slip but it must be noted that feedforward estimate is too idealistic
   an assumption- we are assuming the robot and the surrounding world to behave exactly as 
   we assumed it to be. But odometry relies on more relaxed assumption: That all wheel
   movements actually resulted in corresponding robot movement, which accounts for more
   
   stochastic effects than does our feedforward model.
