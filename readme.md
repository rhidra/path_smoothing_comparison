# Comparison between multiple path planning and path smoothing algorithm

## Random walk
Impossible avec Bezier.

Possible avec Bezier divided, mais les intersections des divisions peuvent avoir
des courbes trop dures. C'est ce qu'on voit dans le diagramme à moustache. Les
extrèmes de la distribution sont très importants.

KF s'en sort pas trop mal. On peut calculer en temps réel l'estimation.
A voir un moyen de quantifier ça.

## RRT*
Pour peu de points, KF permet de corriger la trajectoire sans trop faire de
courbe. On peut sans doute faire du temps réel avec KF. On ne s'éloigne
pas trop du chemin d'origine.

Avec Bezier, on diminue la courbature en rajoutant des points : ajoute du cout
computationnel. De plus, on s'éloigne beaucoup du chemin de départ : fort besoin
de corriger la trajectoire après coup pour éviter les obstacles !

## A*
Résultats similaires à RRT*.

On va stopper les tests avec cet algo parce que c'est pas vraiment intéressant
pour notre application.

## Time performances

La courbe de Bezier normale est super couteuse. Avec ~2000 noeuds, le temps va jusqu'à 100+/-20 secondes.
En comparaison les autres méthodes sont similaires.
L'algorithme a l'air O(n²) ou exponentiel.

Entre Kalman et la courbe de Bezier partagée, Kalman est le moins efficace.
Mais les deux algos ont une complexité qui semble linéaire. Pour 2000 noeuds,
ils restent largement en dessous de 100ms, donc les deux sont satisfaisants
pour du temps réel.  


## References

- State of the art : https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6165411/
- Bezier curve : Bézier curve based smooth path planning for mobile robot, Song et al., 2011.
- Clothoid : http://folk.ntnu.no/skoge/prost/proceedings/ifac11-proceedings/data/html/papers/2944.pdf
