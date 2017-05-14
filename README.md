# Lilium
Projet Géométrie pour la 3D L3S6P17

#### class MeshQuad

- :white_check_mark: clear : vide le maillage
- :white_check_mark: add_vertex : ajoute un sommet
- :white_check_mark: add_quad : ajoute un quad
- :white_check_mark: convert_quads_to_tris : convertit le tableau d'indices de quad en tableau d'indices de triangles.
- :white_check_mark: convert_quads_to_edge : convertit le tableau d'indices de quad en tableau d'indices d'arêtes.
- :white_check_mark: create_cube : creation d'un cube
- :white_check_mark: normal_of_quad : calcul de la normale moyenne à un quad
- :white_check_mark: area_of_quad : calcul de l'aire d'un quad
- :white_check_mark: is_point_in_quad: est-ce qu'un points est dans un quad, le points étant déjà dans le même plan que le quad (considéré comme plan)
- :white_check_mark: intersect_ray_quad: calcul de l'intersection entre un rayon et un quad.
Le rayon est donné par un point et un vecteur, utilise is_point_in_quad
- :white_check_mark: intersected_visible: trouve le quad visible (le plus proche de la caméra) du maillage qui est intersecté par un rayon.
- :white_circle: local_frame: calcule la matrice de transformation qui définit un repère local au milieu de la face,
avec Z aligné sur la normale et X aligné sur la première arête du quad.
- :white_check_mark: extrude_quad: extrude une face quadrangulaire le long de sa normale,
on la décalera d'une distance proportionnelle à la racine carré de son aire.
La face extrudée reste à sa place dans le tableau d'indices (si c'est la ième face, elle reste la ième face)
- :white_check_mark: decal_quad : décale la face suivant la normale d'une distance proportionnelle à la racine carré de son aire
- :white_check_mark: shrink_quad : effectue une homothétie sur la face (centrée sur le centre de la face)
- :white_check_mark: tourne_quad : tourne la face autour de sa normale
