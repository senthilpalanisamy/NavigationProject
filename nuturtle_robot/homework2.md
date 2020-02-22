## Experimental results ##

| Command | Description |
| --- | --- |
| `git status` | List all *new or modified* files |
| `git diff` | Show file differences that **haven't been** staged |


| EXPERIMENT | FV     | AV     | ET    | EX     | EY     | OT   | 
| :___:      | :___:  | :___:  | :___: |:___:   | :___:  | :___:| 
| CCW         | 1     | 2.84   |  0    |    0   |   0    | 1.53 | 
| CCW         |1      |	2.84   |	0    |	0     |	0      |	1.53|	
| CW          |1      |	2.84   |	0    |0	      |0	     |-1.65 |	
| CCW         |0.3    |	0.852  |	0	   |0       |	0      |	1.44|	
| CW          |0.3    |	0.852  |	0    |	0     |	0	     |-1.524|	
| FWD         |1	    |2.84    |	0	   |2	      |0	     |0.221 |	
| BWD         |1	    |2.84    |	0    |	-2    |0	     |0.05  |	
| FWD         |0.3    |	0.852  |	0	   |2	      |0	     |0.02  |	
| BWD         |0.3    | 0.852  |	0	   |-2	    |0	     |-0.03 |


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
