# Lilium
Projet Géométrie pour la 3D L3S6P17

#### class MeshQuad

- :white_check_mark: clear : vide le maillage
- :white_check_mark: add_vertex : ajoute un sommet
- :white_check_mark: add_quad : ajoute un quad
- :white_check_mark: convert_quads_to_tris : convertit le tableau d'indices de quad en tableau d'indices de triangles.
- :white_circle: convert_quads_to_edge : convertit le tableau d'indices de quad en tableau d'indices d'arêtes.
- :white_check_mark: create_cube : creation d'un cube
- :red_circle: normal_of_quad : calcul de la normale moyenne à un quad
- :red_circle: area_of_quad : calcul de l'aire d'un quad
- :red_circle: is_point_in_quad: est-ce qu'un points est dans un quad, le points étant déjà dans le même plan que le quad (considéré comme plan)
- :red_circle: intersect_ray_quad: calcul de l'intersection entre un rayon et un quad. 
Le rayon est donné par un point et un vecteur, utilise is_point_in_quad
- :red_circle: intersected_visible: trouve le quad visible (le plus proche de la caméra) du maillage qui est intersecté par un rayon.
- :red_circle: local_frame: calcule la matrice de transformation qui définit un repère local au milieu de la face,
avec Z aligné sur la normale et X aligné sur la première arête du quad.
- :red_circle: extrude_quad: extrude une face quadrangulaire le long de sa normale,
on la décalera d'une distance proportionnelle à la racine carré de son aire.
La face extrudée reste à sa place dans le tableau d'indices (si c'est la ième face, elle reste la ième face)
- :red_circle: decal_quad : décale la face suivant la normale d'une distance proportionnelle à la racine carré de son aire
- :red_circle: shrink_quad : effectue une homothétie sur la face (centrée sur le centre de la face)
- :red_circle: tourne_quad : tourne la face autour de sa normale
