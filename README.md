# Graphical informatic

This is my projet of graphical informatic in c++. If you want to see the result go in .\Test and open the image.png file.

## Implementation 

Here are the tools that have been implemented in my code : 

Firstly, the mandatory parts are the creation of a clodes scene (6 spheres which allows a closed sapce between each other), the creation of objects (sphere with center, radius and albedo of color) and Ray tracing in order to create the rendering.

Then, I also add shadows by looking t if there is an objects between the object detected and the light.

Then , I add different type of sphere using refraction and reflection.

At this points you could get the following picture : 

![photo_base](/results_rapport/sans_lumiere_indirecte.png)

As you can see, you can get some mirrors spheres and some transparents sphere. In the transparents spehres, you can get a glass like one which will invert the image when you look threw it. The other one is more like a plastic one (a hollow sphere in reality) which will not invert the image.

The problem now is that the shadows are really dark, I've modified it in order to get some smooth shadows using indirect light. Basicelly, I launch 500 rays when I touch an object in order to get a mix of indirect light.

Here you can see the impact of indirect light : 

![indirect](/results_rapport/avec_lumiere_indirect.png)

Then, I have change the source of light from a point a sphere. This allows to get a more realistic image : 

![source](/results_rapport/pas_flou.png)

Note that you can see some pinky reflections in the transparents sphere. This come from the fresnel equation. In fact, in reality a part of the ray is reflected whereas the other part is refracted.

Here is the same image without the implementation of the fresnel equation : 
![fresnel](/results_rapport/sans_fresnel.png)

Finally,I added the blur effect to the image linked to the aperture and focus. Therefore, we can focus on objects nearby or far away. I'm not very satisfied with this part, and I think it can be improved :

![flou](/results_rapport/flou.png)