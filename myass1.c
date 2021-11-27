/* Solution to comp20005 Assignment 1, 2019 semester 1.

   Authorship Declaration:

   (1) I certify that the program contained in this submission is completely
   my own individual work, except where explicitly noted by comments that
   provide details otherwise.  I understand that work that has been developed
   by another student, or by me in collaboration with other students,
   or by non-students as a result of request, solicitation, or payment,
   may not be submitted for assessment in this subject.  I understand that
   submitting for assessment work developed by or in collaboration with
   other students or non-students constitutes Academic Misconduct, and
   may be penalized by mark deductions, or by other penalties determined
   via the University of Melbourne Academic Honesty Policy, as described
   at https://academicintegrity.unimelb.edu.au.

   (2) I also certify that I have not provided a copy of this work in either
   softcopy or hardcopy or any other form to any other student, and nor will
   I do so until after the marks are released. I understand that providing
   my work to other students, regardless of my intention or any undertakings
   made to me by that other student, is also Academic Misconduct.

   (3) I further understand that providing a copy of the assignment
   specification to any form of code authoring or assignment tutoring
   service, or drawing the attention of others to such services and code
   that may have been made available via such a service, may be regarded
   as Student General Misconduct (interfering with the teaching activities
   of the University and/or inciting others to commit Academic Misconduct).
   I understand that an allegation of Student General Misconduct may arise
   regardless of whether or not I personally make use of such solutions
   or sought benefit from such actions.

   Signed by: Goh Xing Yang
   Dated: 15/04/2019

*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define MAXLINES 999      /* max number of data lines in the array */
#define SPEED 4.2	      /* constant speed of the drone (m/s) */
#define DRONEMASS 3.8     /* mass of the drone (KG) */
#define MAXMASS 9.6   	  /* max cumulative mass the drone can fly with (KG) */
#define FULLBATT 100      /* percentage of a fullly charged battery */
#define RANGE_CONST 6300  /* formula constant to calculate the drone range */
#define STAGE2 2	      /* stage 2 to control function execution */
#define STAGE3 3	      /* stage 3 to control function execution */
#define STAGE4 4	      /* stage 4 to control function execution */

/* creating a typedef for the int and double vectors */
typedef double dvector_t[MAXLINES];
typedef int    ivector_t[MAXLINES];

/* function prototypes */
void   initial_data_processing();
void   invalid_data_error();
int    read_into_array(dvector_t X, dvector_t Y, dvector_t KG);
void   check_data(int lines, dvector_t KG);
void   print_read_data(int counter, dvector_t X, dvector_t Y, dvector_t KG);
void   total_mass(dvector_t KG, int lines);
void   int_swap(int *x, int *y);
void   sort_by_distance(dvector_t distance, dvector_t KG, int lines);
void   calculate_distance(dvector_t X, dvector_t Y, dvector_t distance, 
		         	 	  int lines, int stage);
void   battery_usage(dvector_t distance, dvector_t KG, ivector_t order, 
					 int lines, int stage);
double battery_ret(double distance);
double battery_out(double distance, double carry_mass);
double battery_total(double distance, double carry_mass);
int    possible_trip(double distance, double carry_mass, double bat_level);
void   require_replace_battery(double distance, double carry_mass, 
							   double current_bat, double *total_bat,
							   int *bat_count, int stage);
void   ascending_order(ivector_t order, int lines);
void   total_distance(dvector_t distance, int lines, int stage);
double central_point(dvector_t P, int lines);
void   sort_by_delivery(dvector_t distance, dvector_t KG, ivector_t order,
						int lines);
void   re_sort_by_order(ivector_t order, int lines, int i);
int    stage_1(dvector_t X, dvector_t Y, dvector_t KG);
void   stage_2(dvector_t X, dvector_t Y, dvector_t KG, dvector_t distance, 
	  	  	   ivector_t order, int lines, int stage);
void   stage_3(dvector_t KG, dvector_t distance, ivector_t order, int lines,
			   int stage);
void   stage_4(dvector_t X, dvector_t Y, dvector_t KG, dvector_t distance, 
	    	   ivector_t order, int lines, int stage);

/* main binding program
*/
int
main(int argc, char *argv[]) {
	/* creating the arrays for input storage */
	dvector_t X, Y, KG;
	/* creating the arrays to store calculation results */
	dvector_t distance;
	ivector_t order;
	/* creating the buddy variable to count the number of data lines */
	int lines;
	/* main 4-stage program */
	lines=stage_1(X, Y, KG);
	stage_2(X, Y, KG, distance, order, lines, STAGE2);
	stage_3(KG, distance, order, lines, STAGE3);
	stage_4(X, Y, KG, distance, order, lines, STAGE4);
	/* finished! */
	return 0;
}

/* checking initial file format and removing the header line
*/
void
initial_data_processing(){
	char ch;
	double val;
	/* checking if the data file begins with a number (no header), terminating
	if a number is found */
	if (scanf("%lf", &val)==1){
		invalid_data_error();
	}
	/* taking the header line and doing nothing to it (throwing it away) */
	while ((ch=getchar())!= '\n'){
		/* terminating the program if it reaches the end of the file */
		if(ch==EOF){
			invalid_data_error();
		}
	}
}

/* prints error message for invalid data inputs then terminates the program
*/
void
invalid_data_error(){
	printf("Error, data file format is invalid\n");
	printf("The program will now terminate\n");
	exit(EXIT_FAILURE);
}

/* reading the data into the arrays
*/
int
read_into_array(dvector_t X, dvector_t Y, dvector_t KG){
	int counter=0;
	double x, y, kg;
	/* scanning in the values of the array row by row */
	while (scanf("%lf %lf %lf", &x, &y, &kg)==3){
		/* terminating the program if there are more than 999 data lines */
		if (counter==MAXLINES){
			printf("Error, number of data lines exceeds the limit of 999\n");
			printf("The program will now terminate\n");
			exit(EXIT_FAILURE);
		}else {
			X[counter]=x;
			Y[counter]=y;
			KG[counter]=kg;
			counter++;
		}
	}
	return counter;
}

/* checking if the data is valid, terminating if there is any errors
*/
void
check_data(int lines, dvector_t KG){
	char check_c;
	double check_d;
	int i;
	/* terminating if there are no packages */
	if (lines==0){
		invalid_data_error();
	}
	/* checking for extra characters or numbers left in the data file, 
	terminating if any are found */
	if ((scanf("%lf", &check_d)==1) || (scanf("%c", &check_c)==1)){
		/* adding 2 to indicate the exact line of error in the data file */
		printf("Error at line %d of the data file\n", (lines+2));
		printf("The program will now terminate\n");
		exit(EXIT_FAILURE);
	}
	/* checking for any mass of 0 or less, terminating if there are any */
	for (i=0;i<lines;i++){
		if (KG[i]<=0){
			printf("Error, package %d has invalid mass of %.2fkg\n", i, KG[i]);
			printf("The program will now terminate\n");
			exit(EXIT_FAILURE);
		}
	}
}

/* printing the number of data lines, and the first and last data lines
*/
void
print_read_data(int counter, dvector_t X, dvector_t Y, dvector_t KG){
	printf("S1, total data lines:%3d\n", counter);
	/* first element is the dereferenced pointer constant of the array name */
	printf("S1, first data line :  x=%6.1f, y=%6.1f, kg=%4.2f\n", *X, *Y, *KG);
	/* final element is at [counter-1] as array begins at [0] */
	printf("S1, final data line :  x=%6.1f, y=%6.1f, kg=%4.2f\n", X[counter-1],
		   Y[counter-1], KG[counter-1]);
}

/* summing up the masses to deliver
*/
void
total_mass(dvector_t KG, int lines){
	int i;
	double totmass=0;
	/* loop that iterates through the package mass array and adds them up */
	for (i=0;i<lines;i++){
		/* checking if the package exceeds the carrying limit, if it does,
		terminate the program */           
		if ((DRONEMASS+KG[i])<=MAXMASS){
			totmass+=KG[i];
		}else{
			printf("Package number %d is overweight, unable to ", i);
			printf("be delivered\n");
			printf("The program will now terminate\n");
			exit(EXIT_FAILURE);
		}
	}
	printf("S1, total to deliver:%6.2f kg\n", totmass);
}

/* sort the order array in ascending order (0 to lines-1)
*/
void
ascending_order(ivector_t order, int lines){
	int i;
	for (i=0;i<lines;i++){
		order[i]=i;
	}
}	

/* using pythagoras theorem to calculate the distances and store
them in a separate array */
void
calculate_distance(dvector_t X, dvector_t Y, dvector_t distance, int lines,
				   int stage){
	int i;
	/* initial origin set at (0,0) */
	double origin_x=0, origin_y=0;
	/* changes origin point if in stage 4 */
	if (stage==4){
		origin_x=central_point(X, lines);
		origin_y=central_point(Y, lines);
		printf("S%d, centroid location x=%6.1fm, y=%6.1fm\n", stage, origin_x,
			    origin_y);
	}
	for(i=0;i<lines;i++){
		distance[i]=sqrt(pow((X[i]-origin_x),2)+pow((Y[i]-origin_y),2));
	}
}

/* calculating the range of the drone using the given formula
*/
double
drone_range(double carry_mass){	
	return (1.0*RANGE_CONST)/(carry_mass+DRONEMASS);
}

/* checking if the trip is possible to complete with the battery level,
return 0 if impossible and 1 if possible */
int
possible_trip(double distance, double carry_mass, double bat_power){
	if ((battery_total(distance, carry_mass))>bat_power){
		return 0;
	}
	return 1;
}

/* if the trip cannot be completed with the current battery percentage,
use pointers to increase the battery counter and change to a new battery */
void
require_replace_battery(double distance, double carry_mass, double current_bat,
						double *total_bat, int *bat_count, int stage){
	if ((battery_total(distance, carry_mass))>current_bat){
	printf("S%d, change the battery\n", stage);
	*total_bat=100;
	*bat_count+=1;	
	}
}

/* simple calculation for battery usage from origin point to destination
*/
double
battery_out(double distance, double carry_mass){
	return ((distance/drone_range(carry_mass))*100.0);
}

/* simple calculation for battery usage from destination back to origin point,
using the drone_range function with a carry mass of 0kg */
double
battery_ret(double distance){
	return ((distance/drone_range(0))*100.0);
}

/* function that calculates battery usage for the back and fourth trip,
combining battery_out and battery_ret for ease of use */
double
battery_total(double distance, double carry_mass){
	return battery_out(distance, carry_mass)+ battery_ret(distance);
}

/* prints battery usage, uses the order array to control which deliveries
takes place and get printed out */
void
battery_usage(dvector_t distance, dvector_t KG, ivector_t order, int lines,
			  int stage){
	int i, bat_count=1;
	double bat_out, bat_back, total_bat=FULLBATT;
	for (i=0;i<lines;i++){
		/* only print out the battery usage if the trip is possible
		with a fully charged battery */
		if (possible_trip(distance[order[i]], KG[order[i]], FULLBATT)){
			/* check if a battery replacement is required before the trip */
			require_replace_battery(distance[order[i]], KG[order[i]], 
									total_bat, &total_bat, &bat_count, stage);
			/* calculate battery usage for the trip there and back then
			print the battery usage */
			bat_out=battery_out(distance[order[i]], KG[order[i]]);
			bat_back=battery_ret(distance[order[i]]);
			printf("S%d, package=%3d, distance=%6.1fm, ", stage, order[i],
				    distance[order[i]]);
			printf("battery out=%4.1f%%, battery ret=%4.1f%%\n", bat_out,
				    bat_back);
			/* reduce battery percentage by the amount used for the trip */
			total_bat-=(bat_out+bat_back);
		}else{
			/* terminate the program if it is too far away */
			printf("Package number %d is too far away, ", order[i]);
			printf("unable to be delivered\n");
			printf("The program will now terminate\n");
			exit(EXIT_FAILURE);
		}
	}
	/* print the number of batteries required */
	printf("S%d, total batteries required:%4d\n", stage, bat_count);
}

/* calculate the total distance, no checking required as it is already
verified to be a possible delivery */
void
total_distance(dvector_t distance, int lines, int stage){
	int i;
	double tot_dist=0;
	/* trips are always back and fourth to the origin point, so the total
	distance is calculated by multiplying each distance by 2 and adding them
	together */
	for (i=0;i<lines;i++){
		tot_dist+=(2*distance[i]);
	}
	printf("S%d, total flight distance=%6.1f meters, ", stage, tot_dist);
	printf("total flight time=%4.0f seconds\n", tot_dist/SPEED);
}
	
/* sorting the order of delivery, no checking required as they are already 
verified to be possible deliveries */
void
sort_by_delivery(dvector_t distance, dvector_t KG, ivector_t order, int lines){
	int i, j, counter;
	double total_bat;
	/* add i by the counter so the order will not be changed after each loop */
	for (i=0; i<lines; i+=counter){
		/* re-sort the order array in ascending order starting from the i'th
		position (after each batch of delivery order is sorted) */
		re_sort_by_order(order, lines, i);
		counter=1;
		total_bat=FULLBATT-battery_total(distance[order[i]], KG[order[i]]);
		for (j=i+1; j<lines; j++){
			/* checking if any other trip can occur with the current battery */
			if (possible_trip(distance[order[j]], KG[order[j]], total_bat)){
				/* have to decrease the battery if the delivery is possible */
				total_bat-=battery_total(distance[order[j]], KG[order[j]]);
				/* swapping the order array if the delivery is possible */
				int_swap(&order[j], &order[i+counter]);
				/* increase the counter after every possible delivery */
				counter+=1;
			}
		}
	}
}

/* sorting an array in ascending order starting from a given position (i) using
insertion sort */
void
re_sort_by_order(ivector_t order, int lines, int i){
	int k, l;
	for (k=1; k<lines; k++){
		for (l=k-1; (l>=i) && (order[l+1]<order[l]); l--){
			int_swap(&order[l], &order[l+1]);	
		}
	}
}

/* swapping two integers using pointers
*/
void 
int_swap(int *x, int *y){
	int temp;
	temp=*x;
	*x=*y;
	*y=temp;
}

/* calculating the centroid location for the deliveries to begin (stage 4)
*/
double
central_point(dvector_t P, int lines){
	int i;
	double running_sum=0;
	for(i=0;i<lines;i++){
		running_sum+=P[i];
	}
	return ((1.0/lines)*running_sum);
}

/* combined functions to execute stage 1, returning the buddy variable- the 
number of packages scanned
*/
int
stage_1(dvector_t X, dvector_t Y, dvector_t KG){
	int lines;
	initial_data_processing();
	lines=read_into_array(X, Y, KG);
	check_data(lines, KG);
	print_read_data(lines, X, Y, KG);
	total_mass(KG, lines);
	printf("\n");
	return lines;
}

/* combined functions to execute stage 2
*/
void
stage_2(dvector_t X, dvector_t Y, dvector_t KG, dvector_t distance, 
	    ivector_t order, int lines, int stage){
	ascending_order(order, lines);
	calculate_distance(X, Y, distance, lines, stage);
	battery_usage(distance, KG, order, lines, stage);
	total_distance(distance, lines, stage);
	printf("\n");
}

/* combined functions to execute stage 3
*/
void
stage_3(dvector_t KG, dvector_t distance, ivector_t order, int lines,
		int stage){
	sort_by_delivery(distance, KG, order, lines);
	battery_usage(distance, KG, order, lines, stage);
	total_distance(distance, lines, stage);
	printf("\n");
}

/* combined functions to execute stage 4
*/
void
stage_4(dvector_t X, dvector_t Y, dvector_t KG, dvector_t distance, 
	    ivector_t order, int lines, int stage){
	calculate_distance(X, Y, distance, lines, stage);
	/* reset the order array before sorting for new distances */
	ascending_order(order, lines);
	sort_by_delivery(distance, KG, order, lines);
	battery_usage(distance, KG, order, lines, stage);
	total_distance(distance, lines, stage);
	printf("\n");
	printf("Ta daa!\n");
}

/* Programming is fun! */