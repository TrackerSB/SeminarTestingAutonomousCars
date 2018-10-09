# Critical questions

## Data of interest
1. Page 1, right top
    * Mediated perception approaches "**May** add unneccessary complexity to an already difficult task.". ()
    * "Possibly including redundant information"
    * Any idea when it does add complexity and generates redundant information?
2. Page 1, right bottom
    * Behavior reflex approaches "Firstly", "Secondly",...
    * Much more precise than the descriptionn of mediated perception approaches.
3. Page 2, left bottom
    * Affordance indicators
        * Angle of the car relative to the road
        * Distance to lane markings
        * Distance to cars in current and adjacent lanes
4. Page 3, right top
    * Considering highway driving with two or three lanes.
    * Driving alway in the middle
5. Page 4, left middle
    * C coefficient that varies under different driving conditions.
6. Page 4, left middle
    * The baseline `desired_speed` is 72 km/h. (Seems to be the max speed; see )
7. Page 4, left bottom
    * c and d to be calibrated
8. Page 4, right top
    * ConvNet structure
        * 5 convolutional layers
        * 4 fully connected layers with output dimensions [...]
        * (Euclidean loss function used)
9. Page 5, left top
    * "Can not see behind" => Assumption
10. Page 8, left middle
    * "From our experience, the false positive problem can be reduced by simply including more training samples."
11. Page 8, right top
    * "We believe the ConvNet has developed taskspecific features for driving."

## Critical Questions
* Introduction: mediated perception vs behavior relex approaches
    * Based on doi1/doi2
    * Advantages of direct perception against behavior reflex approaches are quite precise whereas advantages against mediated perception approaches are not.
    * In which cases advantages of direct perception appear?
    * It is not stated that direct perception is better in sense of precision but only in a sense like "less complex".
* Introduction: Precision of output
    * Based on doi3/doi4/doi9
    * (Are these three enough in any situation?)
    * Is there any chance it is capable of handling more complex situations than a highway with two or three lanes?
    * Affordance indicators seem not to be enough (Assumption when it is save to move back to the previous lane)
    * **What about extending direct perception to see cars behind?**
        * When switching to lane back, does the assumption hold?
        * They may scale
        * They may overtake
* Mapping from affordance to action: steerCmd definition
    * Based on doi5
    * What conditions do influence C?
    * Handled by ConvNet?
* Some variables not explicitely explained
    * Based on doi6/doi7
    * Why is the baseline `desired_speed` 72 km/h? Is there any certain reason or is it just "We need to fix some value"?
        * It seems it is a result of the size of the input images (280x210) and a distance of 30 meters.
    * Definition 2: c and d to be calibrated?
    * Reasons for choosing exactly this structure?
* Not clarified
    * Based on doi10
    * Which effort has to be taken to decrease the number of false positives and how effective is it?
    * (What taskspecific features?)
