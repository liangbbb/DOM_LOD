#pragma once
#include <string>
#include <vector>
#include <map>
#include <istream>

#include <osg/ref_ptr>
#include <osg/Referenced>
#include <osg/Vec2>
#include <osg/Vec3>
#include <osg/Vec4>

#include <osgDB/ReaderWriter>
#include "gocTriHeader.h"

namespace myobj
{

	class Material
	{
	public:
		Material() :
			ambient(0.2f, 0.2f, 0.2f, 1.0f),
			diffuse(0.8f, 0.8f, 0.8f, 1.0f),
			specular(0.0f, 0.0f, 0.0f, 1.0f),
			emissive(0.0f, 0.0f, 0.0f, 1.0f),
			sharpness(0.0f),
			illum(2),
			Tf(0.0f, 0.0f, 0.0f, 1.0f),
			Ni(0),
			Ns(-1),
			// textureReflection(false),
			alpha(1.0f) {}

		std::string name;

		osg::Vec4   ambient;
		osg::Vec4   diffuse;
		osg::Vec4   specular;
		osg::Vec4   emissive;
		float       sharpness;
		int         illum;            // read but not implemented (would need specific shaders or state manipulation)

		osg::Vec4   Tf;
		int         Ni;
		int         Ns; // shininess 0..1000

		// bool        textureReflection;
		float       alpha;

		class Map
		{
			// -o and -s (offset and scale) options supported for the maps
			// -clamp <on|off> is supported
			// -blendu, -blendv, -imfchan, not supported
			// -mm <base> <gain> is parsed but not actually used
			// -bm <bump_multiplier> is parsed but not used
		public:
			enum TextureMapType {
				DIFFUSE = 0,
				OPACITY,
				AMBIENT,
				SPECULAR,
				SPECULAR_EXPONENT,
				BUMP,
				DISPLACEMENT,
				REFLECTION,        // read of a reflection map will also apply spherical texgen coordinates
				UNKNOWN            // UNKNOWN has to be the last
			};
			Map() :
				type(UNKNOWN),
				name(""),
				uScale(1.0f),
				vScale(1.0f),
				uOffset(0.0f),
				vOffset(0.0f),
				clamp(false) {}


			TextureMapType type;
			std::string name;

			// Texture scale and offset, used for creating the texture matrix.
			// Reader only picks u and v from -s u v w, although all u v and w all need to be specified!
			// e.g. "map_Kd -s u v w <name>" is OK but "map_Kd -s u v <name>" is not, even though tex is only 2D
			float       uScale;
			float       vScale;
			float       uOffset;
			float       vOffset;

			// According to the spec, if clamping is off (default), the effect is a texture repeat
			// if clamping is on, then the effect is a decal texture; i.e. the border is transparent
			bool        clamp;
		};

		std::vector<Map> maps;

	protected:
	};

	class Element : public osg::Referenced
	{
	public:

		typedef std::vector<int> IndexList;

		enum DataType
		{
			POINTS,
			POLYLINE,
			POLYGON
		};

		Element() {}

		Element(DataType type) :
			dataType(type) {}

		enum CoordinateCombination
		{
			VERTICES,
			VERTICES_NORMALS,
			VERTICES_TEXCOORDS,
			VERTICES_NORMALS_TEXCOORDS
		};

		CoordinateCombination getCoordinateCombination() const
		{
			if (vertexIndices.size() == normalIndices.size())
				return (vertexIndices.size() == texCoordIndices.size()) ? VERTICES_NORMALS_TEXCOORDS : VERTICES_NORMALS;
			else
				return (vertexIndices.size() == texCoordIndices.size()) ? VERTICES_TEXCOORDS : VERTICES;
		}

		DataType  dataType;
		IndexList vertexIndices;
		IndexList normalIndices;
		IndexList texCoordIndices;
		IndexList colorsIndices;
	};

	class ElementState
	{
	public:

		ElementState() :
			coordinateCombination(Element::VERTICES),
			smoothingGroup(0) {}

		bool operator < (const ElementState& rhs) const
		{
			if (materialName < rhs.materialName) return true;
			else if (rhs.materialName < materialName) return false;

			if (objectName < rhs.objectName) return true;
			else if (rhs.objectName < objectName) return false;

			if (groupName < rhs.groupName) return true;
			else if (rhs.groupName < groupName) return false;

			if (coordinateCombination < rhs.coordinateCombination) return true;
			else if (rhs.coordinateCombination < coordinateCombination) return false;

			return (smoothingGroup < rhs.smoothingGroup);
		}


		std::string                     objectName;
		std::string                     groupName;
		std::string                     materialName;
		Element::CoordinateCombination  coordinateCombination;
		int                             smoothingGroup;
	};

	class Model
	{
	public:
		Model() :
			currentElementList(0) {}

		void setDatabasePath(const std::string& path) { databasePath = path; }
		const std::string& getDatabasePath() const { return databasePath; }

		std::string lastComponent(const char* linep);
		bool readMTL(std::istream& fin);
		bool readOBJ(std::istream& fin, const osgDB::ReaderWriter::Options* options);

		bool readline(std::istream& fin, char* line, const int LINE_SIZE);
		void addElement(Element* element);

		osg::Vec3 averageNormal(const Element& element) const;
		osg::Vec3 computeNormal(const Element& element) const;
		bool needReverse(const Element& element) const;

		int remapVertexIndex(int vi) { return (vi < 0) ? vertices.size() + vi : vi - 1; }
		int remapNormalIndex(int vi) { return (vi < 0) ? normals.size() + vi : vi - 1; }
		int remapTexCoordIndex(int vi) { return (vi < 0) ? texcoords.size() + vi : vi - 1; }

		typedef std::map<std::string, Material>          MaterialMap;
		typedef std::vector< osg::Vec2 >                Vec2Array;
		typedef std::vector< osg::Vec3 >                Vec3Array;
		typedef std::vector< osg::Vec4 >                Vec4Array;
		typedef std::vector< osg::ref_ptr<Element> >    ElementList;
		typedef std::map< ElementState, ElementList >    ElementStateMap;


		std::string     databasePath;
		MaterialMap     materialMap;

		Vec3Array       vertices;
		Vec4Array       colors;
		Vec3Array       normals;
		Vec2Array       texcoords;

		ElementState    currentElementState;

		ElementStateMap elementStateMap;
		ElementList*    currentElementList;

	};

}

namespace mypol
{
	typedef struct polHeader	//头区
	{
		char format_version[64];
		char user_comments[128];
		char dummy1[4];
		unsigned int nv;                //顶点数
		unsigned int vblock_length;		//顶点数据块字节长度
		unsigned int np;				//多边形数
		unsigned int pblock_length;     //多边形数据块字节长度
		unsigned int color_texture_flag;//彩色纹理标识
		unsigned int ctblock_length;	//彩色或纹理数据块字节长度
		unsigned int ngr;				//分组数
		unsigned int grblock_length;	//分组数据块字节长度
		unsigned int dummy2[71];
	}pol_header;

	typedef struct polGroup //组头区
	{
		unsigned int poly_begin;	//本组多边形起始号
		unsigned int poly_end;		//本组多边形结束号
		unsigned int mat_default;
		float mat_diffuse[4];
		float mat_ambient[3];
		float mat_specular[3];
		float mat_emissive[3];
		float mat_shininess;
		char name[256];
		char texmap[256];
	}polGroup;

	class Material
	{
	public:
		Material() :
			ambient(0.2f, 0.2f, 0.2f, 1.0f),
			diffuse(0.8f, 0.8f, 0.8f, 1.0f),
			specular(0.0f, 0.0f, 0.0f, 1.0f),
			emissive(0.0f, 0.0f, 0.0f, 1.0f),
			sharpness(0.0f),
			illum(2),
			Tf(0.0f, 0.0f, 0.0f, 1.0f),
			Ni(0),
			Ns(-1),
			// textureReflection(false),
			alpha(1.0f) {}

		std::string name;

		osg::Vec4   ambient;
		osg::Vec4   diffuse;
		osg::Vec4   specular;
		osg::Vec4   emissive;
		float       sharpness;
		int         illum;            // read but not implemented (would need specific shaders or state manipulation)

		osg::Vec4   Tf;
		int         Ni;
		int         Ns; // shininess 0..1000

		// bool        textureReflection;
		float       alpha;


		class Map
		{
			// -o and -s (offset and scale) options supported for the maps
			// -clamp <on|off> is supported
			// -blendu, -blendv, -imfchan, not supported
			// -mm <base> <gain> is parsed but not actually used
			// -bm <bump_multiplier> is parsed but not used
		public:
			enum TextureMapType {
				DIFFUSE = 0,
				OPACITY,
				AMBIENT,
				SPECULAR,
				SPECULAR_EXPONENT,
				BUMP,
				DISPLACEMENT,
				REFLECTION,        // read of a reflection map will also apply spherical texgen coordinates
				UNKNOWN            // UNKNOWN has to be the last
			};
			Map() :
				type(UNKNOWN),
				name(""),
				uScale(1.0f),
				vScale(1.0f),
				uOffset(0.0f),
				vOffset(0.0f),
				clamp(false) {}


			TextureMapType type;
			std::string name;

			// Texture scale and offset, used for creating the texture matrix.
			// Reader only picks u and v from -s u v w, although all u v and w all need to be specified!
			// e.g. "map_Kd -s u v w <name>" is OK but "map_Kd -s u v <name>" is not, even though tex is only 2D
			float       uScale;
			float       vScale;
			float       uOffset;
			float       vOffset;

			// According to the spec, if clamping is off (default), the effect is a texture repeat
			// if clamping is on, then the effect is a decal texture; i.e. the border is transparent
			bool        clamp;
		};

		std::vector<Map> maps;

	protected:
	};

	class Element : public osg::Referenced
	{
	public:

		typedef std::vector<int> IndexList;

		enum DataType
		{
			POINTS,
			POLYLINE,
			POLYGON
		};

		Element() {};
		Element(DataType type) :
			dataType(type) {}

		enum CoordinateCombination
		{
			VERTICES,
			VERTICES_NORMALS,
			VERTICES_TEXCOORDS,
			VERTICES_NORMALS_TEXCOORDS
		};

		CoordinateCombination getCoordinateCombination() const
		{
			if (vertexIndices.size() == normalIndices.size())
				return (vertexIndices.size() == texCoordIndices.size()) ? VERTICES_NORMALS_TEXCOORDS : VERTICES_NORMALS;
			else
				return (vertexIndices.size() == texCoordIndices.size()) ? VERTICES_TEXCOORDS : VERTICES;
		}

		DataType  dataType;
		IndexList vertexIndices;
		IndexList normalIndices;
		IndexList texCoordIndices;
		IndexList colorsIndices;
	};

	class ElementState
	{
	public:

		ElementState() :
			coordinateCombination(Element::VERTICES),
			smoothingGroup(0) {}

		bool operator < (const ElementState& rhs) const
		{
			if (materialName < rhs.materialName) return true;
			else if (rhs.materialName < materialName) return false;

			if (objectName < rhs.objectName) return true;
			else if (rhs.objectName < objectName) return false;

			if (groupName < rhs.groupName) return true;
			else if (rhs.groupName < groupName) return false;

			if (coordinateCombination < rhs.coordinateCombination) return true;
			else if (rhs.coordinateCombination < coordinateCombination) return false;

			return (smoothingGroup < rhs.smoothingGroup);
		}


		std::string                     objectName;
		std::string                     groupName;
		std::string                     materialName;
		Element::CoordinateCombination  coordinateCombination;
		int                             smoothingGroup;
	};

	class Model
	{
	public:
		Model() :
			currentElementList(0) {}

		void setDatabasePath(const std::string& path) { databasePath = path; }
		const std::string& getDatabasePath() const { return databasePath; }

		std::string lastComponent(const char* linep);
		bool readMTL(std::istream& fin);//?
		void addElement(Element* element);//?
		void SwapForBigEndian(unsigned char* theArry, int tupleSiz, int numTuple);//二进制编码方式修改：对Big Endian 编码进行位序交换操作，得到Little Endian
		void cuNormals(osg::Vec3f * pVertex, unsigned int nVertices, TriangleIndex_Record_With_TextureIndex * pTinID, unsigned int start, unsigned int end, unsigned inttextureUnit);
		void cuNormal(float v1[3], float v2[3], float v3[3], double n[3]);
		void computeNormalDirection(float v1[3], float v2[3], float v3[3], double n[3]);
		bool readline(std::istream& fin, char* line, const int LINE_SIZE);

		bool readPOL(std::string filedir, const osgDB::ReaderWriter::Options* options);//？
		osg::Vec3 averageNormal(const Element& element) const;
		osg::Vec3 computeNormal(const Element& element) const;
		bool needReverse(const Element& element) const;

		int remapVertexIndex(int vi) { return (vi < 0) ? vertices.size() + vi : vi - 1; }
		int remapNormalIndex(int vi) { return (vi < 0) ? normals.size() + vi : vi - 1; }
		int remapTexCoordIndex(int vi) { return (vi < 0) ? texcoords.size() + vi : vi - 1; }

		typedef std::map<std::string, Material>          MaterialMap;
		typedef std::vector< osg::Vec2 >                Vec2Array;
		typedef std::vector< osg::Vec3 >                Vec3Array;
		typedef std::vector< osg::Vec4 >                Vec4Array;
		typedef std::vector< osg::ref_ptr<Element> >    ElementList;
		typedef std::map< ElementState, ElementList >    ElementStateMap;


		std::string     databasePath;
		MaterialMap     materialMap;

		Vec3Array       vertices;
		Vec4Array       colors;
		Vec3Array       normals;
		Vec2Array       texcoords;

		ElementState    currentElementState;

		ElementStateMap elementStateMap;
		ElementList*    currentElementList;

	};

}
