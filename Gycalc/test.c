#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#define ZERO_ERROR -6.20
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define MAX_VALUES 100000

int main() 
{
    // Define the variables to store readings
    float pitch;
    float fac = 0.0, facavg = 0.0;
    int count = 1, tag, throttletheo, throttlereal;

    int frequency[MAX_VALUES] = {0};
    float most_frequent_fac = 0.0;
    int mode_count = 0;

    // Open the file in read mode
    FILE *file = fopen("readings1.txt", "r");
    if (file == NULL) 
    {
        printf("Error opening file!\n");                
        return 1;
    }

    // Read the data from the file using proper fscanf and end-of-file check
    while (fscanf(file, " I (%d) example: Throttle: %d | Pitch: %f", &tag, &throttlereal, &pitch) == 3) // I (14884) example: Throttle: 640 | Pitch: 48.50
    {
        if(throttlereal != 0 && pitch != ZERO_ERROR)
        {
            // pitch to radians
            pitch -= ZERO_ERROR;
            pitch = pitch * (M_PI / 180.0);

            // calculating real throttle
            throttletheo = (int)(cos(pitch) * 2.504);

            fac = (float) throttlereal / throttletheo;
            facavg = ((facavg * (count - 1)) + fac) / count;

            fac = round(fac * 10) / 10;
            count++;

            if (fac >= 0.0 && fac <= 10000.0) 
            {
                // Calculate the index for the frequency array
                int index = (int)(fac * 10); // Multiply by 10 to get the index for one decimal place

                // Update the frequency
                frequency[index]++;

                // Update the mode (most frequent rounded fac value)
                if (frequency[index] > mode_count) 
                {
                    mode_count = frequency[index];
                    most_frequent_fac = (float)index / 10.0;
                }
            }
        }
    }

    // Close the file
    fclose(file);

    // Print the calculated results
    printf("Average: %0.2f\n", facavg);
    printf("Most frequent value: %.1f (appeared %d times)\n", most_frequent_fac, mode_count);

    return 0;
}
