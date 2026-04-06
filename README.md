Sensors = {s1, s2, s3, s4, s5, s6}

Curves:- s2/s5 detects line. bot goes on drifting to left/right. error increases slowly.
	PID controls it.  
	
Sharp turns(90):- edge sensors(s1/s6) detects line. then all sesnsors=70.error spikes, all sensors on white.Bot overshoot itself if speed not controlled. Keep a track of bot last pos after edge sensors detection bot must turn hard right/left.

Discontinous Line :- moving on line suddenly all sensors=70.after some time again line is detected.Make the bot move based on its last pos(last error) if straight go straight or right goes right.  

T(J)-junction && Cross:- sudden sensors=0. best way is ignore the detection and move based on last pos.
	Can keep a track of it like #1 time sensors=0 act as if it is T-junction, for #3 act as cross. keep a count of junction keep on increasing it after every detection(manual method we need to add those movements after seeing track). 
	Better way is to freeze PID for some time and react based on situation and then start pid again. Manage sensors reading as it can be overflowed and reading can get corrupted. (in Uno no issue as 32kb of ram) 

Diodes:- all black -- \ or / -- all black -- line resumed. 
	2 cases:
		1. (/)center lights first,then edges: (s1--s2--s3) || (s6--s5--s4)
		2. (\)edge lights first,then center: (s3--s2--s1) || (s4--s5--s6)

Distractions from main track:- 

How to distinguish betn sharp turns and discontinous line?
	Almost same case as sensors=255 suddenly. For sharp turns last pos edge sensors detects line. For discontinous line after gap covers again detects line.
