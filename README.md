# SteadyAffineTransforms

The is the preliminary implementation of "Steady Affine Motions and Morphs" by Jarek Rossignac paper. Given
a 3D object and the Affinity Matrix A, it calculates the intermediate positions of the model without unexpected
behavior.

**Acknowledgemt**
I received great help from Prof. Shizuo Kaji from Kyushu University. He explained me how to use AffineLib library.


**External libraries:**
1. AffineLib
2. ICP: (Iterative Closest Points)
3. libQGLViewer

**Compilation:**
Presently, there is a Makefile in the distribution which the user needs to modify.
After proper paths to various libraries have been set, just write "make" on the
command line and the executab;e "sam" will be generated.

**Usages**
1. Given a mesh model M, translate and rotate it to some desired position. 
2. Using ICP calculate the Affinity Matrix "A" and store it in "model.xf".
3. Use command line
       sam model.off model.xf
4. Press "N" to see the next position of the model.



