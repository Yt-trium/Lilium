#include "meshquad.h"
#include "matrices.h"

MeshQuad::MeshQuad():
	m_nb_ind_edges(0)
{

}


void MeshQuad::gl_init()
{
	m_shader_flat = new ShaderProgramFlat();
	m_shader_color = new ShaderProgramColor();

	//VBO
	glGenBuffers(1, &m_vbo);

	//VAO
	glGenVertexArrays(1, &m_vao);
	glBindVertexArray(m_vao);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glEnableVertexAttribArray(m_shader_flat->idOfVertexAttribute);
	glVertexAttribPointer(m_shader_flat->idOfVertexAttribute, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glBindVertexArray(0);

	glGenVertexArrays(1, &m_vao2);
	glBindVertexArray(m_vao2);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glEnableVertexAttribArray(m_shader_color->idOfVertexAttribute);
	glVertexAttribPointer(m_shader_color->idOfVertexAttribute, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glBindVertexArray(0);


	//EBO indices
	glGenBuffers(1, &m_ebo);
	glGenBuffers(1, &m_ebo2);
}

void MeshQuad::gl_update()
{
	//VBO
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * m_points.size() * sizeof(GLfloat), &(m_points[0][0]), GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);


	std::vector<int> tri_indices;
	convert_quads_to_tris(m_quad_indices,tri_indices);

	//EBO indices
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER,tri_indices.size() * sizeof(int), &(tri_indices[0]), GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);


	std::vector<int> edge_indices;
	convert_quads_to_edges(m_quad_indices,edge_indices);
	m_nb_ind_edges = edge_indices.size();

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo2);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER,m_nb_ind_edges * sizeof(int), &(edge_indices[0]), GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}



void MeshQuad::set_matrices(const Mat4& view, const Mat4& projection)
{
	viewMatrix = view;
	projectionMatrix = projection;
}

void MeshQuad::draw(const Vec3& color)
{

	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0f, 1.0f);

	m_shader_flat->startUseProgram();
	m_shader_flat->sendViewMatrix(viewMatrix);
	m_shader_flat->sendProjectionMatrix(projectionMatrix);
	glUniform3fv(m_shader_flat->idOfColorUniform, 1, glm::value_ptr(color));
	glBindVertexArray(m_vao);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_ebo);
	glDrawElements(GL_TRIANGLES, 3*m_quad_indices.size()/2,GL_UNSIGNED_INT,0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
	glBindVertexArray(0);
	m_shader_flat->stopUseProgram();

	glDisable(GL_POLYGON_OFFSET_FILL);

	m_shader_color->startUseProgram();
	m_shader_color->sendViewMatrix(viewMatrix);
	m_shader_color->sendProjectionMatrix(projectionMatrix);
	glUniform3f(m_shader_color->idOfColorUniform, 0.0f,0.0f,0.0f);
	glBindVertexArray(m_vao2);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_ebo2);
	glDrawElements(GL_LINES, m_nb_ind_edges,GL_UNSIGNED_INT,0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
	glBindVertexArray(0);
	m_shader_color->stopUseProgram();
}

void MeshQuad::clear()
{
	m_points.clear();
	m_quad_indices.clear();
}

int MeshQuad::add_vertex(const Vec3& P)
{
    m_points.push_back(P);
    return m_points.size()-1;
}


void MeshQuad::add_quad(int i1, int i2, int i3, int i4)
{
    m_quad_indices.push_back(i1);
    m_quad_indices.push_back(i2);
    m_quad_indices.push_back(i3);
    m_quad_indices.push_back(i4);
}

void MeshQuad::convert_quads_to_tris(const std::vector<int>& quads, std::vector<int>& tris)
{
	tris.clear();
	tris.reserve(3*quads.size()/2); // 1 quad = 4 indices -> 2 tris = 6 indices d'ou ce calcul (attention division entiere)

	// Pour chaque quad on genere 2 triangles
	// Attention a respecter l'orientation des triangles

    for(unsigned int i=0;i<quads.size();i+=4)
    {
        tris.push_back(quads.at(i));
        tris.push_back(quads.at(i+1));
        tris.push_back(quads.at(i+2));

        tris.push_back(quads.at(i));
        tris.push_back(quads.at(i+2));
        tris.push_back(quads.at(i+3));
    }
}

void MeshQuad::convert_quads_to_edges(const std::vector<int>& quads, std::vector<int>& edges)
{
	edges.clear();
	edges.reserve(quads.size()); // ( *2 /2 !)

	// Pour chaque quad on genere 4 aretes, 1 arete = 2 indices.
	// Mais chaque arete est commune a 2 quads voisins !
	// Comment n'avoir qu'une seule fois chaque arete ?

    for(unsigned int i=0;i<quads.size();i+=4)
    {
        edges.push_back(quads.at(i));
        edges.push_back(quads.at(i+1));

        edges.push_back(quads.at(i+1));
        edges.push_back(quads.at(i+2));

        edges.push_back(quads.at(i+2));
        edges.push_back(quads.at(i+3));

        edges.push_back(quads.at(i+3));
        edges.push_back(quads.at(i));

    }
}


void MeshQuad::create_cube()
{
	clear();
	// ajouter 8 sommets (-1 +1)

    add_vertex(Vec3(-1,-1, 1));
    add_vertex(Vec3( 1,-1, 1));
    add_vertex(Vec3( 1, 1, 1));
    add_vertex(Vec3(-1, 1, 1));

    add_vertex(Vec3(-1,-1,-1));
    add_vertex(Vec3( 1,-1,-1));
    add_vertex(Vec3( 1, 1,-1));
    add_vertex(Vec3(-1, 1,-1));


    // ajouter 6 faces (sens trigo)
    add_quad(0,1,2,3);
    add_quad(5,4,7,6);

    add_quad(0,3,7,4);
    add_quad(1,5,6,2);

    add_quad(3,2,6,7);
    add_quad(1,0,4,5);

	gl_update();
}

Vec3 MeshQuad::normal_of_quad(const Vec3& A, const Vec3& B, const Vec3& C, const Vec3& D)
{
	// Attention a l'ordre des points !
	// le produit vectoriel n'est pas commutatif U ^ V = - V ^ U
	// ne pas oublier de normaliser le resultat.

    return vec_normalize(vec_cross((D-A),(B-A)) + vec_cross((A-B),(C-B)) + vec_cross((B-C),(D-C)) + vec_cross((C-D),(A-D)));
}

float MeshQuad::area_of_quad(const Vec3& A, const Vec3& B, const Vec3& C, const Vec3& D)
{
    /*
     * Calcul de l'air du quad même si celui ci n'est pas dans un seul plan.
     * Décomposition du quad en 2 triangles
     * Calcul de l'air des triangle puis addition.
     */
    // Triangle 1 ABC
    float ABC = 0.5*vec_length(vec_cross((B-A),(C-A)));

    // Triangle 2 ACD
    float ABD = 0.5*vec_length(vec_cross((C-A),(D-A)));

    return ABC+ABD;

	// aire du quad - aire tri + aire tri

    // aire du tri = 1/2 aire parallelogramme

	// aire parallelogramme: cf produit vectoriel
}


bool MeshQuad::is_points_in_quad(const Vec3& P, const Vec3& A, const Vec3& B, const Vec3& C, const Vec3& D)
{
	// On sait que P est dans le plan du quad.

	// P est-il au dessus des 4 plans contenant chacun la normale au quad et une arete AB/BC/CD/DA ?
    // si oui il est dans le quad

    auto is_point_up = [&] (Vec3 u, Vec3 v, Vec3 A, Vec3 P) -> float
    {
        float a = (u.y*v.z-u.z*v.y);
        float b = (u.z*v.x-u.x*v.z);
        float c = (u.x*v.y-u.y*v.x);
        float d = -(a*A.x+b*A.y+c*A.z);
        return (a*P.x + b*P.y + c*P.z + d);
    };

    Vec3 N = normal_of_quad(A,B,C,D);

    Vec3 AB = B-A;
    Vec3 BC = C-B;
    Vec3 CD = D-C;
    Vec3 DA = A-D;

    float P1 =  is_point_up(AB,N,A,P);
    float P2 =  is_point_up(BC,N,B,P);
    float P3 =  is_point_up(CD,N,C,P);
    float P4 =  is_point_up(DA,N,D,P);

    return std::signbit(-P1) && std::signbit(-P2) && std::signbit(-P3) && std::signbit(-P4);
}

bool MeshQuad::intersect_ray_quad(const Vec3& P, const Vec3& Dir, int q, Vec3& inter)
{
    // recuperation des indices de points
    int id = q*4;
	// recuperation des points
    Vec3 A = m_points.at(this->m_quad_indices.at(id++));
    Vec3 B = m_points.at(this->m_quad_indices.at(id++));
    Vec3 C = m_points.at(this->m_quad_indices.at(id++));
    Vec3 D = m_points.at(this->m_quad_indices.at(id++));

    // calcul de l'equation du plan (N+d)
    Vec3 N = normal_of_quad(A,B,C,D);

    /* P : N.x * x + N.y * y + C.z * z + d = 0 */
    float a = N.x;
    float b = N.y;
    float c = N.z;
    float d = -(N.x * A.x + N.y * A.y + N.z * A.z);

    /*
    qDebug() << -(N.x * A.x + N.y * A.y + N.z * A.z);
    qDebug() << -(N.x * B.x + N.y * B.y + N.z * B.z);
    qDebug() << -(N.x * C.x + N.y * C.y + N.z * C.z);
    qDebug() << -(N.x * D.x + N.y * A.y + N.z * D.z);
    */

	// calcul de l'intersection rayon plan
	// I = P + alpha*Dir est dans le plan => calcul de alpha

	// alpha => calcul de I

    float alpha = -(d + a*P.x + b*P.y + c*P.z) / (a*Dir.x + b*Dir.y + c*Dir.z);

    inter = Vec3(alpha * Dir.x + P.x,alpha * Dir.y + P.y,alpha * Dir.z + P.z);

    if(alpha > 1 || alpha < 0)
        return false;

    // I dans le quad ?
    return is_points_in_quad(inter,A,B,C,D);
}


int MeshQuad::intersected_visible(const Vec3& P, const Vec3& Dir)
{
	// on parcours tous les quads
	// on teste si il y a intersection avec le rayon
	// on garde le plus proche (de P)

	int inter = -1;
    float d;
    float tmp;
    Vec3 I;

    for(int i=0;i<m_quad_indices.size()/4;i++)
    {
        if(intersect_ray_quad(P,Dir,i,I))
        {
            tmp = vec_length(P-I);

            if(inter == -1)
            {
                d = tmp;
                inter = i;
            }
            else if(tmp < d)
            {
                d = tmp;
                inter = i;
            }
        }
    }

	return inter;
}


Mat4 MeshQuad::local_frame(int q)
{
    // qDebug() << "local_frame" << q;

	// Repere locale = Matrice de transfo avec
	// les trois premieres colones: X,Y,Z locaux
	// la derniere colonne l'origine du repere

    // ici Z = N et X = AB
    // Origine le centre de la face
    // longueur des axes : [AB]/2

	// recuperation des indices de points
    int id = q*4;

    // recuperation des points
    Vec3 A = m_points.at(this->m_quad_indices.at(id++));
    Vec3 B = m_points.at(this->m_quad_indices.at(id++));
    Vec3 C = m_points.at(this->m_quad_indices.at(id++));
    Vec3 D = m_points.at(this->m_quad_indices.at(id++));

    // calcul de Z:N puis de X:arete on en deduit Y
    Vec3 Z = normal_of_quad(A,B,C,D);
    Vec3 X = B-A;
    Vec3 Y = vec_cross(Z,X);

	// calcul du centre
    Vec3 center = (A + B + C + D);
    center /= 4;

	// calcul de la taille
    float length = vec_length(B-A);

    // calcul de la matrice

    // PAS ENCORE IMPLEMENTE
    float RX = 0.0;
    float RY = 0.0;
    float RZ = 0.0;

    Mat4 g1 = rotateX(RX) * rotateY(RY) * rotateZ(RZ) * translate(center.x,center.y,center.z) * scale(length,length,length);

    Mat4 g2 = Mat4(Vec4(-X,0),Vec4(-Y,0),Vec4(-Z,0),Vec4(center,1));

    return g2;
}

void MeshQuad::extrude_quad(int q)
{
    // qDebug() << "extrude_quad" << q;

	// recuperation des indices de points
    int id = q*4;

	// recuperation des points
    int IDA = m_quad_indices.at(id);
    int IDB = m_quad_indices.at(id+1);
    int IDC = m_quad_indices.at(id+2);
    int IDD = m_quad_indices.at(id+3);

    Vec3 A = m_points.at(IDA);
    Vec3 B = m_points.at(IDB);
    Vec3 C = m_points.at(IDC);
    Vec3 D = m_points.at(IDD);

	// calcul de la normale
    Vec3 N = normal_of_quad(A,B,C,D);

	// calcul de la hauteur
    float distance = sqrt(area_of_quad(A,B,C,D));

    // calcul et ajout des 4 nouveaux points
    Vec3 NPA = A - N*distance;
    Vec3 NPB = B - N*distance;
    Vec3 NPC = C - N*distance;
    Vec3 NPD = D - N*distance;

    int IDNPA = add_vertex(NPA);
    int IDNPB = add_vertex(NPB);
    int IDNPC = add_vertex(NPC);
    int IDNPD = add_vertex(NPD);

    // on remplace le quad initial par le quad du dessu
    m_quad_indices[id]   = IDNPA;
    m_quad_indices[id+1] = IDNPB;
    m_quad_indices[id+2] = IDNPC;
    m_quad_indices[id+3] = IDNPD;

	// on ajoute les 4 quads des cotes
    add_quad(IDB,IDNPB,IDNPA,IDA);
    add_quad(IDC,IDNPC,IDNPB,IDB);
    add_quad(IDD,IDNPD,IDNPC,IDC);
    add_quad(IDA,IDNPA,IDNPD,IDD);

	gl_update();
}


void MeshQuad::decale_quad(int q, float d)
{
    // qDebug() << "decale_quad" << q << d;

	// recuperation des indices de points
    int id = q*4;
    int IDA = m_quad_indices.at(id);
    int IDB = m_quad_indices.at(id+1);
    int IDC = m_quad_indices.at(id+2);
    int IDD = m_quad_indices.at(id+3);

	// recuperation des (references de) points
    Vec3 A = m_points.at(IDA);
    Vec3 B = m_points.at(IDB);
    Vec3 C = m_points.at(IDC);
    Vec3 D = m_points.at(IDD);

	// calcul de la normale
    Vec3 N = normal_of_quad(A,B,C,D);
    float distance = sqrt(area_of_quad(A,B,C,D)) * d;

    // modification des points
    m_points[IDA] += N*-distance;
    m_points[IDB] += N*-distance;
    m_points[IDC] += N*-distance;
    m_points[IDD] += N*-distance;

	gl_update();
}


void MeshQuad::shrink_quad(int q, float s)
{
    // qDebug() << "shrink_quad" << q << s;

	// recuperation des indices de points
    int id = q*4;
    int IDA = m_quad_indices.at(id);
    int IDB = m_quad_indices.at(id+1);
    int IDC = m_quad_indices.at(id+2);
    int IDD = m_quad_indices.at(id+3);

	// recuperation des (references de) points
    Vec3 A = m_points.at(IDA);
    Vec3 B = m_points.at(IDB);
    Vec3 C = m_points.at(IDC);
    Vec3 D = m_points.at(IDD);

	// ici  pas besoin de passer par une matrice
	// calcul du centre
    Vec3 center = (A + B + C + D);
    center /= 4;

	 // modification des points
    m_points[IDA] += (center-A)*s;
    m_points[IDB] += (center-B)*s;
    m_points[IDC] += (center-C)*s;
    m_points[IDD] += (center-D)*s;

	gl_update();
}


void MeshQuad::tourne_quad(int q, float a)
{
    qDebug() << "tourne_quad" << q << a;

	// recuperation des indices de points
    int id = q*4;
    int IDA = m_quad_indices.at(id);
    int IDB = m_quad_indices.at(id+1);
    int IDC = m_quad_indices.at(id+2);
    int IDD = m_quad_indices.at(id+3);

	// recuperation des (references de) points
    Vec3 A = m_points.at(IDA);
    Vec3 B = m_points.at(IDB);
    Vec3 C = m_points.at(IDC);
    Vec3 D = m_points.at(IDD);

    // calcul du centre et de la normal
    Vec3 center = (A + B + C + D);
    center /= 4;
    Vec3 N = normal_of_quad(A,B,C,D);

	// generation de la matrice de transfo:
    Mat4 transfo = glm::rotate(a,N);

	// tourne autour du Z de la local frame
	// indice utilisation de glm::inverse()

    // Application au 4 points du quad

    auto rotate = [&] (Vec3 V, Vec3 center, Mat4 transfo) -> Vec3
    {
        Vec4 X = Vec4(V,1);
        X -= Vec4(center, 0);
        X  = transfo * X;
        X += Vec4(center, 0);
        return Vec3(X.x,X.y,X.z);
    };

    m_points[IDA] = rotate(A,center,transfo);
    m_points[IDB] = rotate(B,center,transfo);
    m_points[IDC] = rotate(C,center,transfo);
    m_points[IDD] = rotate(D,center,transfo);

    gl_update();
}

void MeshQuad::create_fig1()
{
    qDebug() << "create_fig1";

    clear();
    create_cube();

    int beg=m_quad_indices.size()/4-6;
    int end=m_quad_indices.size()/4;

    for(int c=0;c<32;c++)
    for(int i=beg;i<end;i++)
    {
        extrude_quad(i);
        tourne_quad(i,0.1);
        shrink_quad(i,0.1);
    }

    gl_update();
}

void MeshQuad::create_fig2()
{
    qDebug() << "create_fig2";

    clear();
    create_cube();

    fig2(3);

    gl_update();
}

void MeshQuad::fig2(int r)
{
    if(r==0)
        return;
    else
    {
        int beg=m_quad_indices.size()/4-6;
        int end=m_quad_indices.size()/4;

        for(int i=beg;i<end;i++)
        {
            extrude_quad(i);
            shrink_quad(i,0.8);
            extrude_quad(i);
            shrink_quad(i,0.3);
            fig2(r-1);
        }
    }
}

void MeshQuad::create_fig3()
{
    qDebug() << "create_fig3";

    clear();
    create_cube();

    int beg=m_quad_indices.size()/4-6;
    int end=m_quad_indices.size()/4;

    for(int c=0;c<32;c++)
    for(int i=beg;i<end;i++)
    {
        extrude_quad(i);
        shrink_quad(i,(c%2==1?0.5:-0.5));
        shrink_quad(i,(c%2==1?0.5:-0.5));
        decale_quad(i,(c%2==0?0.5:-0.5));
    }

    gl_update();
}

void MeshQuad::debug_print_Vec3(Vec3 A)
{
    // qDebug() << "debug_print_Vec3";
    // qDebug() << "x =" << A.x <<  " y =" << A.y <<  " z =" << A.z;
    qDebug() << "(" << A.x << "," << A.y << "," << A.z << ")";
}

void MeshQuad::debug_test_normal_of_quad()
{
    qDebug() << "debug_test_normal_of_quad";
    qDebug() << "(0,0,-1) =";
    debug_print_Vec3(normal_of_quad(Vec3(-1,-1,0),Vec3(1,-1,0),Vec3(1,1,0),Vec3(-1,1,0)));
    qDebug() << "(0,0,1) =";
    debug_print_Vec3(normal_of_quad(Vec3(-1,-1,0),Vec3(-1,1,0),Vec3(1,1,0),Vec3(1,-1,0)));
}

void MeshQuad::debug_test_area_of_quad()
{
    qDebug() << "debug_test_area_of_quad";
    qDebug() << "18.32 ==" << area_of_quad(Vec3(-2.72,4.18, 1),Vec3(2.48,4.5, 1),Vec3(2.48, 1.1, 1),Vec3(1.56,-2, 1));
    qDebug() << "24.31 ==" << area_of_quad(Vec3(-2.72,4.18, 0),Vec3(2.48,4.5, 1),Vec3(2.48, 1.1, -1.45),Vec3(1.56,-2,2));
}

void MeshQuad::debug_test_is_points_in_quad()
{
    qDebug() << "debug_test_is_points_in_quad";
    qDebug() << "true  ==" << is_points_in_quad(Vec3(0,0,0),Vec3(-1,-1,0),Vec3(1,-1,0),Vec3(1,1,0),Vec3(-1,1,0));
    qDebug() << "true  ==" << is_points_in_quad(Vec3(0.5,-0.5,0),Vec3(-1,-1,0),Vec3(1,-1,0),Vec3(1,1,0),Vec3(-1,1,0));
    qDebug() << "true  ==" << is_points_in_quad(Vec3(-0.5, 0.5,0),Vec3(-1,-1,0),Vec3(1,-1,0),Vec3(1,1,0),Vec3(-1,1,0));
    qDebug() << "true  ==" << is_points_in_quad(Vec3(0.5, 0.5,0),Vec3(-1,-1,0),Vec3(1,-1,0),Vec3(1,1,0),Vec3(-1,1,0));

    qDebug() << "false ==" << is_points_in_quad(Vec3(2,0,0),Vec3(-1,-1,0),Vec3(1,-1,0),Vec3(1,1,0),Vec3(-1,1,0));
    qDebug() << "false ==" << is_points_in_quad(Vec3(12,12,0),Vec3(-1,-1,0),Vec3(1,-1,0),Vec3(1,1,0),Vec3(-1,1,0));
}

void MeshQuad::debug_test_intersect_ray_quad()
{
    qDebug() << "debug_test_intersect_ray_quad";
    Vec3 inter(0,0,0);
    qDebug() << "true  ==" << intersect_ray_quad(Vec3(0,0,2),Vec3(0,0,-4),0,inter);
    debug_print_Vec3(inter);
            qDebug() << " == ";
    debug_print_Vec3(Vec3(0,0,1));

    qDebug() << "true  ==" << intersect_ray_quad(Vec3(0,0,0),Vec3(0.5,0.5,2),0,inter);
    debug_print_Vec3(inter);
            qDebug() << " == ";
    debug_print_Vec3(Vec3(0.25,0.25,1));

    qDebug() << "false ==" << intersect_ray_quad(Vec3(0,0,3),Vec3(0,0,-1),0,inter);

    qDebug() << "false ==" << intersect_ray_quad(Vec3(0,0,2),Vec3(0,0,1),0,inter);

    qDebug() << "false ==" << intersect_ray_quad(Vec3(4,4,2),Vec3(0,0,-4),0,inter);
}

void MeshQuad::debug_test_intersected_visible()
{
    qDebug() << "debug_test_intersected_visible";
    qDebug() << "-1 ==" << intersected_visible(Vec3(10,10,10),Vec3(1,1,1));
    qDebug() << " 0 ==" << intersected_visible(Vec3(0,0,2),Vec3(0,0,-10));
    qDebug() << " 1 ==" << intersected_visible(Vec3(0,0,0.5),Vec3(0,0,-10));
}
