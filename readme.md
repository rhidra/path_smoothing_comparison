# Comparison between multiple path planning and path smoothing algorithm

Random walk :
Impossible avec Bezier.
KF s'en sort pas trop mal. On peut calculer en temps réel l'estimation. A voir un moyen de quantifier ça.

RRT* :
Pour peu de points, KF permet de corriger la trajectoire sans trop faire de courbe.
Avec Bezier, on diminue la courbature en rajoutant des points : ajoute du cout computationnel.



State of the art : https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6165411/
Bezier curve : Bézier curve based smooth path planning for mobile robot, Song et al., 2011.
Clothoid : http://folk.ntnu.no/skoge/prost/proceedings/ifac11-proceedings/data/html/papers/2944.pdf
