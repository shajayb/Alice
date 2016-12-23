#ifndef _MATRICES_

/////////////////////////////////////////////////////////////////////////////
//// 2x2 matrix
/////////////////////////////////////////////////////////////////////////////
class Matrix2
{
public:
	// constructors
	Matrix2();  // init with identity
	Matrix2(const float src[4]);
	Matrix2(float xx, float xy, float yx, float yy);

	void        set(const float src[4]);
	void        set(float xx, float xy, float yx, float yy);
	void        setRow(int index, const float row[2]);
	void        setRow(int index, const vec& v);
	void        setColumn(int index, const float col[2]);
	void        setColumn(int index, const vec& v);

	const float* get() const;
	float       getDeterminant();

	Matrix2&    identity();
	Matrix2&    transpose();                            // transpose itself and return reference
	Matrix2&    invert();

	// operators
	Matrix2     operator+(const Matrix2& rhs) const;    // add rhs
	Matrix2     operator-(const Matrix2& rhs) const;    // subtract rhs
	Matrix2&    operator+=(const Matrix2& rhs);         // add rhs and update this object
	Matrix2&    operator-=(const Matrix2& rhs);         // subtract rhs and update this object
	vec         operator*(const vec& rhs) const;    // multiplication: v' = M * v
	Matrix2     operator*(const Matrix2& rhs) const;    // multiplication: M3 = M1 * M2
	Matrix2&    operator*=(const Matrix2& rhs);         // multiplication: M1' = M1 * M2
	bool        operator==(const Matrix2& rhs) const;   // exact compare, no epsilon
	bool        operator!=(const Matrix2& rhs) const;   // exact compare, no epsilon
	float       operator[](int index) const;            // subscript operator v[0], v[1]
	float&      operator[](int index);                  // subscript operator v[0], v[1]

	friend Matrix2 operator-(const Matrix2& m);                     // unary operator (-)
	friend Matrix2 operator*(float scalar, const Matrix2& m);       // pre-multiplication
	friend vec operator*(const vec& vec, const Matrix2& m); // pre-multiplication
	friend std::ostream& operator<<(std::ostream& os, const Matrix2& m);

protected:

private:
	float m[4];

};



///////////////////////////////////////////////////////////////////////////
// 3x3 matrix
///////////////////////////////////////////////////////////////////////////
class Matrix3
{
public:
	// constructors
	Matrix3();  // init with identity
	Matrix3(const float src[9]);
	Matrix3(float xx, float xy, float xz,
		float yx, float yy, float yz,
		float zx, float zy, float zz);

	void        set(const float src[9]);
	void        set(float xx, float xy, float xz,
		float yx, float yy, float yz,
		float zx, float zy, float zz);
	void        setRow(int index, const float row[3]);
	void        setRow(int index, const vec& v);
	void        setColumn(int index, const float col[3]);
	void        setColumn(int index, const vec& v);
	vec			getColumn(int index);

	const float* get() const;
	float       getDeterminant();

	Matrix3&    identity();
	Matrix3&    transpose();                            // transpose itself and return reference
	Matrix3&    invert();

	// operators
	Matrix3     operator+(const Matrix3& rhs) const;    // add rhs
	Matrix3     operator-(const Matrix3& rhs) const;    // subtract rhs
	Matrix3&    operator+=(const Matrix3& rhs);         // add rhs and update this object
	Matrix3&    operator-=(const Matrix3& rhs);         // subtract rhs and update this object
	vec			operator*(const vec& rhs) const;		// multiplication: v' = M * v
	Matrix3     operator*(const Matrix3& rhs) const;    // multiplication: M3 = M1 * M2
	Matrix3&    operator*=(const Matrix3& rhs);         // multiplication: M1' = M1 * M2
	bool        operator==(const Matrix3& rhs) const;   // exact compare, no epsilon
	bool        operator!=(const Matrix3& rhs) const;   // exact compare, no epsilon
	float       operator[](int index) const;            // subscript operator v[0], v[1]
	float&      operator[](int index);                  // subscript operator v[0], v[1]

	friend Matrix3 operator-(const Matrix3& m);                     // unary operator (-)
	friend Matrix3 operator*(float scalar, const Matrix3& m);       // pre-multiplication
	friend vec operator*(const vec& vec, const Matrix3& m); // pre-multiplication
	friend std::ostream& operator<<(std::ostream& os, const Matrix3& m);

protected:

private:
	float m[9];

};



///////////////////////////////////////////////////////////////////////////
// 4x4 matrix
///////////////////////////////////////////////////////////////////////////
class Matrix4
{
public:
	// constructors
	Matrix4();  // init with identity
	Matrix4(const float src[16]);
	Matrix4(float xx, float xy, float xz, float xw,
		float yx, float yy, float yz, float yw,
		float zx, float zy, float zz, float zw,
		float wx, float wy, float wz, float ww);

	void        set(const float src[16]);
	void        set(float xx, float xy, float xz, float xw,
		float yx, float yy, float yz, float yw,
		float zx, float zy, float zz, float zw,
		float wx, float wy, float wz, float ww);
	void        setRow(int index, const float row[4]);
	void        setRow(int index, const Vector4& v);
	void        setRow(int index, const vec& v);
	void        setColumn(int index, const float col[4]);
	void        setColumn(int index, const Vector4& v);
	void        setColumn(int index, const vec& v);
	vec			getColumn(int index);
	void		getBasisVectors(vec &x, vec &y, vec &z, vec &cen);

	const float* get() const;
	const float* getTranspose();                        // return transposed matrix
	float        getDeterminant();

	Matrix4&    identity();
	Matrix4&    transpose();                            // transpose itself and return reference
	Matrix4&    invert();                               // check best inverse method before inverse
	Matrix4&    invertEuclidean();                      // inverse of Euclidean transform matrix
	Matrix4&    invertAffine();                         // inverse of affine transform matrix
	Matrix4&    invertProjective();                     // inverse of projective matrix using partitioning
	Matrix4&    invertGeneral();                        // inverse of generic matrix

	// transform matrix
	Matrix4&    translate(float x, float y, float z);   // translation by (x,y,z)
	Matrix4&    translate(const vec& v);            //
	Matrix4&    rotate(float angle, const vec& axis); // rotate angle(degree) along the given axix
	Matrix4&    rotate(float angle, float x, float y, float z);
	Matrix4&    rotateX(float angle);                   // rotate on X-axis with degree
	Matrix4&    rotateY(float angle);                   // rotate on Y-axis with degree
	Matrix4&    rotateZ(float angle);                   // rotate on Z-axis with degree
	Matrix4&    scale(float scale);                     // uniform scale
	Matrix4&    scale(float sx, float sy, float sz);    // scale by (sx, sy, sz) on each axis

	Matrix4& setDiag(float s);
	// operators
	Matrix4     operator+(const Matrix4& rhs) const;    // add rhs
	Matrix4     operator-(const Matrix4& rhs) const;    // subtract rhs
	Matrix4&    operator+=(const Matrix4& rhs);         // add rhs and update this object
	Matrix4&    operator-=(const Matrix4& rhs);         // subtract rhs and update this object
	Vector4     operator*(const Vector4& rhs) const;    // multiplication: v' = M * v
	vec     operator*(const vec& rhs) const;    // multiplication: v' = M * v
	Matrix4     operator*(const Matrix4& rhs) const;    // multiplication: M3 = M1 * M2
	Matrix4&    operator*=(const Matrix4& rhs);         // multiplication: M1' = M1 * M2
	bool        operator==(const Matrix4& rhs) const;   // exact compare, no epsilon
	bool        operator!=(const Matrix4& rhs) const;   // exact compare, no epsilon
	float       operator[](int index) const;            // subscript operator v[0], v[1]
	float&      operator[](int index);                  // subscript operator v[0], v[1]

	friend Matrix4 operator-(const Matrix4& m);                     // unary operator (-)
	friend Matrix4 operator*(float scalar, const Matrix4& m);       // pre-multiplication
	friend vec operator*(const vec& vec, const Matrix4& m); // pre-multiplication
	friend Vector4 operator*(const Vector4& vec, const Matrix4& m); // pre-multiplication
	friend std::ostream& operator<<(std::ostream& os, const Matrix4& m);

//protected:

//private:
	float       getCofactor(float m0, float m1, float m2,
		float m3, float m4, float m5,
		float m6, float m7, float m8);

	float m[16];
	float tm[16];                                       // transpose m

};



///////////////////////////////////////////////////////////////////////////
// inline functions for Matrix2
///////////////////////////////////////////////////////////////////////////
inline Matrix2::Matrix2()
{
	// initially identity matrix
	identity();
}



inline Matrix2::Matrix2(const float src[4])
{
	set(src);
}



inline Matrix2::Matrix2(float xx, float xy, float yx, float yy)
{
	set(xx, xy, yx, yy);
}



inline void Matrix2::set(const float src[4])
{
	m[0] = src[0];  m[1] = src[1];  m[2] = src[2];  m[3] = src[3];
}



inline void Matrix2::set(float xx, float xy, float yx, float yy)
{
	m[0] = xx;  m[1] = xy;  m[2] = yx;  m[3] = yy;
}



inline void Matrix2::setRow(int index, const float row[2])
{
	m[index * 2] = row[0];  m[index * 2 + 1] = row[1];
}



inline void Matrix2::setRow(int index, const vec& v)
{
	m[index * 2] = v.x;  m[index * 2 + 1] = v.y;
}



inline void Matrix2::setColumn(int index, const float col[2])
{
	m[index] = col[0];  m[index + 2] = col[1];
}



inline void Matrix2::setColumn(int index, const vec& v)
{
	m[index] = v.x;  m[index + 2] = v.y;
}



inline const float* Matrix2::get() const
{
	return m;
}



inline Matrix2& Matrix2::identity()
{
	m[0] = m[3] = 1.0f;
	m[1] = m[2] = 0.0f;
	return *this;
}



inline Matrix2 Matrix2::operator+(const Matrix2& rhs) const
{
	return Matrix2(m[0] + rhs[0], m[1] + rhs[1], m[2] + rhs[2], m[3] + rhs[3]);
}



inline Matrix2 Matrix2::operator-(const Matrix2& rhs) const
{
	return Matrix2(m[0] - rhs[0], m[1] - rhs[1], m[2] - rhs[2], m[3] - rhs[3]);
}



inline Matrix2& Matrix2::operator+=(const Matrix2& rhs)
{
	m[0] += rhs[0];  m[1] += rhs[1];  m[2] += rhs[2];  m[3] += rhs[3];
	return *this;
}



inline Matrix2& Matrix2::operator-=(const Matrix2& rhs)
{
	m[0] -= rhs[0];  m[1] -= rhs[1];  m[2] -= rhs[2];  m[3] -= rhs[3];
	return *this;
}



inline vec Matrix2::operator*(const vec& rhs) const
{
	return vec(m[0] * rhs.x + m[1] * rhs.y, m[2] * rhs.x + m[3] * rhs.y, 0);
}



inline Matrix2 Matrix2::operator*(const Matrix2& rhs) const
{
	return Matrix2(m[0] * rhs[0] + m[1] * rhs[2], m[0] * rhs[1] + m[1] * rhs[3],
		m[2] * rhs[0] + m[3] * rhs[2], m[2] * rhs[1] + m[3] * rhs[3]);
}



inline Matrix2& Matrix2::operator*=(const Matrix2& rhs)
{
	*this = *this * rhs;
	return *this;
}



inline bool Matrix2::operator==(const Matrix2& rhs) const
{
	return (m[0] == rhs[0]) && (m[1] == rhs[1]) && (m[2] == rhs[2]) && (m[3] == rhs[3]);
}



inline bool Matrix2::operator!=(const Matrix2& rhs) const
{
	return (m[0] != rhs[0]) || (m[1] != rhs[1]) || (m[2] != rhs[2]) || (m[3] != rhs[3]);
}



inline float Matrix2::operator[](int index) const
{
	return m[index];
}



inline float& Matrix2::operator[](int index)
{
	return m[index];
}



inline Matrix2 operator-(const Matrix2& rhs)
{
	return Matrix2(-rhs[0], -rhs[1], -rhs[2], -rhs[3]);
}



inline Matrix2 operator*(float s, const Matrix2& rhs)
{
	return Matrix2(s*rhs[0], s*rhs[1], s*rhs[2], s*rhs[3]);
}



inline vec operator*(const vec& v, const Matrix2& rhs)
{
	return vec(v.x*rhs[0] + v.y*rhs[2], v.x*rhs[1] + v.y*rhs[3], 0);
}



inline std::ostream& operator<<(std::ostream& os, const Matrix2& m)
{
	os << "(" << m[0] << ",\t" << m[1] << ")\n"
		<< "(" << m[2] << ",\t" << m[3] << ")\n";
	return os;
}
// END OF MATRIX2 INLINE //////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////
// inline functions for Matrix3
///////////////////////////////////////////////////////////////////////////
inline Matrix3::Matrix3()
{
	// initially identity matrix
	identity();
}



inline Matrix3::Matrix3(const float src[9])
{
	set(src);
}



inline Matrix3::Matrix3(float xx, float xy, float xz,
	float yx, float yy, float yz,
	float zx, float zy, float zz)
{
	set(xx, xy, xz, yx, yy, yz, zx, zy, zz);
}



inline void Matrix3::set(const float src[9])
{
	m[0] = src[0];  m[1] = src[1];  m[2] = src[2];
	m[3] = src[3];  m[4] = src[4];  m[5] = src[5];
	m[6] = src[6];  m[7] = src[7];  m[8] = src[8];
}



inline void Matrix3::set(float xx, float xy, float xz,
	float yx, float yy, float yz,
	float zx, float zy, float zz)
{
	m[0] = xx;  m[1] = xy;  m[2] = xz;
	m[3] = yx;  m[4] = yy;  m[5] = yz;
	m[6] = zx;  m[7] = zy;  m[8] = zz;
}



inline void Matrix3::setRow(int index, const float row[3])
{
	m[index * 3] = row[0];  m[index * 3 + 1] = row[1];  m[index * 3 + 2] = row[2];
}



inline void Matrix3::setRow(int index, const vec& v)
{
	m[index * 3] = v.x;  m[index * 3 + 1] = v.y;  m[index * 3 + 2] = v.z;
}



inline void Matrix3::setColumn(int index, const float col[3])
{
	m[index] = col[0];  m[index + 3] = col[1];  m[index + 6] = col[2];
}



inline void Matrix3::setColumn(int index, const vec& v)
{
	m[index] = v.x;  m[index + 3] = v.y;  m[index + 6] = v.z;
}

inline vec Matrix3::getColumn(int index)
{
	vec ret(m[index], m[index + 3], m[index + 6]);
	return ret;
}

inline const float* Matrix3::get() const
{
	return m;
}



inline Matrix3& Matrix3::identity()
{
	m[0] = m[4] = m[8] = 1.0f;
	m[1] = m[2] = m[3] = m[5] = m[6] = m[7] = 0.0f;
	return *this;
}



inline Matrix3 Matrix3::operator+(const Matrix3& rhs) const
{
	return Matrix3(m[0] + rhs[0], m[1] + rhs[1], m[2] + rhs[2],
		m[3] + rhs[3], m[4] + rhs[4], m[5] + rhs[5],
		m[6] + rhs[6], m[7] + rhs[7], m[8] + rhs[8]);
}



inline Matrix3 Matrix3::operator-(const Matrix3& rhs) const
{
	return Matrix3(m[0] - rhs[0], m[1] - rhs[1], m[2] - rhs[2],
		m[3] - rhs[3], m[4] - rhs[4], m[5] - rhs[5],
		m[6] - rhs[6], m[7] - rhs[7], m[8] - rhs[8]);
}



inline Matrix3& Matrix3::operator+=(const Matrix3& rhs)
{
	m[0] += rhs[0];  m[1] += rhs[1];  m[2] += rhs[2];
	m[3] += rhs[3];  m[4] += rhs[4];  m[5] += rhs[5];
	m[6] += rhs[6];  m[7] += rhs[7];  m[8] += rhs[8];
	return *this;
}



inline Matrix3& Matrix3::operator-=(const Matrix3& rhs)
{
	m[0] -= rhs[0];  m[1] -= rhs[1];  m[2] -= rhs[2];
	m[3] -= rhs[3];  m[4] -= rhs[4];  m[5] -= rhs[5];
	m[6] -= rhs[6];  m[7] -= rhs[7];  m[8] -= rhs[8];
	return *this;
}



inline vec Matrix3::operator*(const vec& rhs) const
{
	return vec(m[0] * rhs.x + m[1] * rhs.y + m[2] * rhs.z,
		m[3] * rhs.x + m[4] * rhs.y + m[5] * rhs.z,
		m[6] * rhs.x + m[7] * rhs.y + m[8] * rhs.z);
}



inline Matrix3 Matrix3::operator*(const Matrix3& rhs) const
{
	return Matrix3(m[0] * rhs[0] + m[1] * rhs[3] + m[2] * rhs[6], m[0] * rhs[1] + m[1] * rhs[4] + m[2] * rhs[7], m[0] * rhs[2] + m[1] * rhs[5] + m[2] * rhs[8],
		m[3] * rhs[0] + m[4] * rhs[3] + m[5] * rhs[6], m[3] * rhs[1] + m[4] * rhs[4] + m[5] * rhs[7], m[3] * rhs[2] + m[4] * rhs[5] + m[5] * rhs[8],
		m[6] * rhs[0] + m[7] * rhs[3] + m[8] * rhs[6], m[6] * rhs[1] + m[7] * rhs[4] + m[8] * rhs[7], m[6] * rhs[2] + m[7] * rhs[5] + m[8] * rhs[8]);
}



inline Matrix3& Matrix3::operator*=(const Matrix3& rhs)
{
	*this = *this * rhs;
	return *this;
}



inline bool Matrix3::operator==(const Matrix3& rhs) const
{
	return (m[0] == rhs[0]) && (m[1] == rhs[1]) && (m[2] == rhs[2]) &&
		(m[3] == rhs[3]) && (m[4] == rhs[4]) && (m[5] == rhs[5]) &&
		(m[6] == rhs[6]) && (m[7] == rhs[7]) && (m[8] == rhs[8]);
}



inline bool Matrix3::operator!=(const Matrix3& rhs) const
{
	return (m[0] != rhs[0]) || (m[1] != rhs[1]) || (m[2] != rhs[2]) ||
		(m[3] != rhs[3]) || (m[4] != rhs[4]) || (m[5] != rhs[5]) ||
		(m[6] != rhs[6]) || (m[7] != rhs[7]) || (m[8] != rhs[8]);
}



inline float Matrix3::operator[](int index) const
{
	return m[index];
}



inline float& Matrix3::operator[](int index)
{
	return m[index];
}



inline Matrix3 operator-(const Matrix3& rhs)
{
	return Matrix3(-rhs[0], -rhs[1], -rhs[2], -rhs[3], -rhs[4], -rhs[5], -rhs[6], -rhs[7], -rhs[8]);
}



inline Matrix3 operator*(float s, const Matrix3& rhs)
{
	return Matrix3(s*rhs[0], s*rhs[1], s*rhs[2], s*rhs[3], s*rhs[4], s*rhs[5], s*rhs[6], s*rhs[7], s*rhs[8]);
}



inline vec operator*(const vec& v, const Matrix3& m)
{
	return vec(v.x*m[0] + v.y*m[3] + v.z*m[6], v.x*m[1] + v.y*m[4] + v.z*m[7], v.x*m[2] + v.y*m[5] + v.z*m[8]);
}



inline std::ostream& operator<<(std::ostream& os, const Matrix3& m)
{
	os << "(" << m[0] << ",\t" << m[1] << ",\t" << m[2] << ")\n"
		<< "(" << m[3] << ",\t" << m[4] << ",\t" << m[5] << ")\n"
		<< "(" << m[6] << ",\t" << m[7] << ",\t" << m[8] << ")\n";
	return os;
}
// END OF MATRIX3 INLINE //////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////
// inline functions for Matrix4
///////////////////////////////////////////////////////////////////////////
inline Matrix4::Matrix4()
{
	// initially identity matrix
	identity();
}



inline Matrix4::Matrix4(const float src[16])
{
	set(src);
}



inline Matrix4::Matrix4(float xx, float xy, float xz, float xw,
	float yx, float yy, float yz, float yw,
	float zx, float zy, float zz, float zw,
	float wx, float wy, float wz, float ww)
{
	set(xx, xy, xz, xw, yx, yy, yz, yw, zx, zy, zz, zw, wx, wy, wz, ww);
}



inline void Matrix4::set(const float src[16])
{
	m[0] = src[0];  m[1] = src[1];  m[2] = src[2];  m[3] = src[3];
	m[4] = src[4];  m[5] = src[5];  m[6] = src[6];  m[7] = src[7];
	m[8] = src[8];  m[9] = src[9];  m[10] = src[10]; m[11] = src[11];
	m[12] = src[12]; m[13] = src[13]; m[14] = src[14]; m[15] = src[15];
}



inline void Matrix4::set(float xx, float xy, float xz, float xw,
	float yx, float yy, float yz, float yw,
	float zx, float zy, float zz, float zw,
	float wx, float wy, float wz, float ww)
{
	m[0] = xx;  m[1] = xy;  m[2] = xz;  m[3] = xw;
	m[4] = yx;  m[5] = yy;  m[6] = yz;  m[7] = yw;
	m[8] = zx;  m[9] = zy;  m[10] = zz;  m[11] = zw;
	m[12] = wx;  m[13] = wy;  m[14] = wz;  m[15] = ww;
}



inline void Matrix4::setRow(int index, const float row[4])
{
	m[index * 4] = row[0];  m[index * 4 + 1] = row[1];  m[index * 4 + 2] = row[2];  m[index * 4 + 3] = row[3];
}



inline void Matrix4::setRow(int index, const Vector4& v)
{
	m[index * 4] = v.x;  m[index * 4 + 1] = v.y;  m[index * 4 + 2] = v.z;  m[index * 4 + 3] = v.w;
}



inline void Matrix4::setRow(int index, const vec& v)
{
	m[index * 4] = v.x;  m[index * 4 + 1] = v.y;  m[index * 4 + 2] = v.z;
}



inline void Matrix4::setColumn(int index, const float col[4])
{
	m[index] = col[0];  m[index + 4] = col[1];  m[index + 8] = col[2];  m[index + 12] = col[3];
}



inline void Matrix4::setColumn(int index, const Vector4& v)
{
	m[index] = v.x;  m[index + 4] = v.y;  m[index + 8] = v.z;  m[index + 12] = v.w;
}



inline void Matrix4::setColumn(int index, const vec& v)
{
	m[index] = v.x;  m[index + 4] = v.y;  m[index + 8] = v.z;
}

inline vec Matrix4::getColumn(int index)
{
	return vec(m[index], m[index + 4], m[index + 8]);
}

inline void Matrix4::getBasisVectors(vec &x, vec &y, vec &z, vec &cen)
{
	x = getColumn(0); y = getColumn(1); z = getColumn(2); cen = getColumn(3);

}

inline const float* Matrix4::get() const
{
	return m;
}



inline const float* Matrix4::getTranspose()
{
	tm[0] = m[0];   tm[1] = m[4];   tm[2] = m[8];   tm[3] = m[12];
	tm[4] = m[1];   tm[5] = m[5];   tm[6] = m[9];   tm[7] = m[13];
	tm[8] = m[2];   tm[9] = m[6];   tm[10] = m[10];  tm[11] = m[14];
	tm[12] = m[3];   tm[13] = m[7];   tm[14] = m[11];  tm[15] = m[15];
	return tm;
}



inline Matrix4& Matrix4::identity()
{
	m[0] = m[5] = m[10] = m[15] = 1.0f;
	m[1] = m[2] = m[3] = m[4] = m[6] = m[7] = m[8] = m[9] = m[11] = m[12] = m[13] = m[14] = 0.0f;
	return *this;
}



inline Matrix4 Matrix4::operator+(const Matrix4& rhs) const
{
	return Matrix4(m[0] + rhs[0], m[1] + rhs[1], m[2] + rhs[2], m[3] + rhs[3],
		m[4] + rhs[4], m[5] + rhs[5], m[6] + rhs[6], m[7] + rhs[7],
		m[8] + rhs[8], m[9] + rhs[9], m[10] + rhs[10], m[11] + rhs[11],
		m[12] + rhs[12], m[13] + rhs[13], m[14] + rhs[14], m[15] + rhs[15]);
}



inline Matrix4 Matrix4::operator-(const Matrix4& rhs) const
{
	return Matrix4(m[0] - rhs[0], m[1] - rhs[1], m[2] - rhs[2], m[3] - rhs[3],
		m[4] - rhs[4], m[5] - rhs[5], m[6] - rhs[6], m[7] - rhs[7],
		m[8] - rhs[8], m[9] - rhs[9], m[10] - rhs[10], m[11] - rhs[11],
		m[12] - rhs[12], m[13] - rhs[13], m[14] - rhs[14], m[15] - rhs[15]);
}



inline Matrix4& Matrix4::operator+=(const Matrix4& rhs)
{
	m[0] += rhs[0];    m[1] += rhs[1];    m[2] += rhs[2];    m[3] += rhs[3];
	m[4] += rhs[4];    m[5] += rhs[5];    m[6] += rhs[6];    m[7] += rhs[7];
	m[8] += rhs[8];    m[9] += rhs[9];    m[10] += rhs[10];  m[11] += rhs[11];
	m[12] += rhs[12];  m[13] += rhs[13];  m[14] += rhs[14];  m[15] += rhs[15];
	return *this;
}



inline Matrix4& Matrix4::operator-=(const Matrix4& rhs)
{
	m[0] -= rhs[0];    m[1] -= rhs[1];    m[2] -= rhs[2];    m[3] -= rhs[3];
	m[4] -= rhs[4];    m[5] -= rhs[5];    m[6] -= rhs[6];    m[7] -= rhs[7];
	m[8] -= rhs[8];    m[9] -= rhs[9];    m[10] -= rhs[10];  m[11] -= rhs[11];
	m[12] -= rhs[12];  m[13] -= rhs[13];  m[14] -= rhs[14];  m[15] -= rhs[15];
	return *this;
}



inline Vector4 Matrix4::operator*(const Vector4& rhs) const
{
	return Vector4(m[0] * rhs.x + m[1] * rhs.y + m[2] * rhs.z + m[3] * rhs.w,
		m[4] * rhs.x + m[5] * rhs.y + m[6] * rhs.z + m[7] * rhs.w,
		m[8] * rhs.x + m[9] * rhs.y + m[10] * rhs.z + m[11] * rhs.w,
		m[12] * rhs.x + m[13] * rhs.y + m[14] * rhs.z + m[15] * rhs.w);
}



inline vec Matrix4::operator*(const vec& rhs) const
{
	return vec(m[0] * rhs.x + m[1] * rhs.y + m[2] * rhs.z + m[3] * 1.0,
		m[4] * rhs.x + m[5] * rhs.y + m[6] * rhs.z + m[7] * 1.0,
		m[8] * rhs.x + m[9] * rhs.y + m[10] * rhs.z + m[11] * 1.0);
}



inline Matrix4 Matrix4::operator*(const Matrix4& n) const
{
	return Matrix4(m[0] * n[0] + m[1] * n[4] + m[2] * n[8] + m[3] * n[12], m[0] * n[1] + m[1] * n[5] + m[2] * n[9] + m[3] * n[13], m[0] * n[2] + m[1] * n[6] + m[2] * n[10] + m[3] * n[14], m[0] * n[3] + m[1] * n[7] + m[2] * n[11] + m[3] * n[15],
		m[4] * n[0] + m[5] * n[4] + m[6] * n[8] + m[7] * n[12], m[4] * n[1] + m[5] * n[5] + m[6] * n[9] + m[7] * n[13], m[4] * n[2] + m[5] * n[6] + m[6] * n[10] + m[7] * n[14], m[4] * n[3] + m[5] * n[7] + m[6] * n[11] + m[7] * n[15],
		m[8] * n[0] + m[9] * n[4] + m[10] * n[8] + m[11] * n[12], m[8] * n[1] + m[9] * n[5] + m[10] * n[9] + m[11] * n[13], m[8] * n[2] + m[9] * n[6] + m[10] * n[10] + m[11] * n[14], m[8] * n[3] + m[9] * n[7] + m[10] * n[11] + m[11] * n[15],
		m[12] * n[0] + m[13] * n[4] + m[14] * n[8] + m[15] * n[12], m[12] * n[1] + m[13] * n[5] + m[14] * n[9] + m[15] * n[13], m[12] * n[2] + m[13] * n[6] + m[14] * n[10] + m[15] * n[14], m[12] * n[3] + m[13] * n[7] + m[14] * n[11] + m[15] * n[15]);
}



inline Matrix4& Matrix4::operator*=(const Matrix4& rhs)
{
	*this = *this * rhs;
	return *this;
}



inline bool Matrix4::operator==(const Matrix4& n) const
{
	return (m[0] == n[0]) && (m[1] == n[1]) && (m[2] == n[2]) && (m[3] == n[3]) &&
		(m[4] == n[4]) && (m[5] == n[5]) && (m[6] == n[6]) && (m[7] == n[7]) &&
		(m[8] == n[8]) && (m[9] == n[9]) && (m[10] == n[10]) && (m[11] == n[11]) &&
		(m[12] == n[12]) && (m[13] == n[13]) && (m[14] == n[14]) && (m[15] == n[15]);
}



inline bool Matrix4::operator!=(const Matrix4& n) const
{
	return (m[0] != n[0]) || (m[1] != n[1]) || (m[2] != n[2]) || (m[3] != n[3]) ||
		(m[4] != n[4]) || (m[5] != n[5]) || (m[6] != n[6]) || (m[7] != n[7]) ||
		(m[8] != n[8]) || (m[9] != n[9]) || (m[10] != n[10]) || (m[11] != n[11]) ||
		(m[12] != n[12]) || (m[13] != n[13]) || (m[14] != n[14]) || (m[15] != n[15]);
}



inline float Matrix4::operator[](int index) const
{
	return m[index];
}



inline float& Matrix4::operator[](int index)
{
	return m[index];
}



inline Matrix4 operator-(const Matrix4& rhs)
{
	return Matrix4(-rhs[0], -rhs[1], -rhs[2], -rhs[3], -rhs[4], -rhs[5], -rhs[6], -rhs[7], -rhs[8], -rhs[9], -rhs[10], -rhs[11], -rhs[12], -rhs[13], -rhs[14], -rhs[15]);
}



inline Matrix4 operator*(float s, const Matrix4& rhs)
{
	return Matrix4(s*rhs[0], s*rhs[1], s*rhs[2], s*rhs[3], s*rhs[4], s*rhs[5], s*rhs[6], s*rhs[7], s*rhs[8], s*rhs[9], s*rhs[10], s*rhs[11], s*rhs[12], s*rhs[13], s*rhs[14], s*rhs[15]);
}



inline Vector4 operator*(const Vector4& v, const Matrix4& m)
{
	return Vector4(v.x*m[0] + v.y*m[4] + v.z*m[8] + v.w*m[12], v.x*m[1] + v.y*m[5] + v.z*m[9] + v.w*m[13], v.x*m[2] + v.y*m[6] + v.z*m[10] + v.w*m[14], v.x*m[3] + v.y*m[7] + v.z*m[11] + v.w*m[15]);
}



inline vec operator*(const vec& v, const Matrix4& m)
{
	return vec(v.x*m[0] + v.y*m[4] + v.z*m[8], v.x*m[1] + v.y*m[5] + v.z*m[9], v.x*m[2] + v.y*m[6] + v.z*m[10]);
}



inline std::ostream& operator<<(std::ostream& os, const Matrix4& m)
{
	os << "(" << m[0] << ",\t" << m[1] << ",\t" << m[2] << ",\t" << m[3] << ")\n"
		<< "(" << m[4] << ",\t" << m[5] << ",\t" << m[6] << ",\t" << m[7] << ")\n"
		<< "(" << m[8] << ",\t" << m[9] << ",\t" << m[10] << ",\t" << m[11] << ")\n"
		<< "(" << m[12] << ",\t" << m[13] << ",\t" << m[14] << ",\t" << m[15] << ")\n";
	return os;
}
// END OF MATRIX4 INLINE //////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// return the determinant of 2x2 matrix
///////////////////////////////////////////////////////////////////////////////
float Matrix2::getDeterminant()
{
	return m[0] * m[3] - m[1] * m[2];
}



///////////////////////////////////////////////////////////////////////////////
// inverse of 2x2 matrix
// If cannot find inverse, set identity matrix
///////////////////////////////////////////////////////////////////////////////
Matrix2& Matrix2::invert()
{
	float determinant = m[0] * m[3] - m[1] * m[2];
	if (fabs(determinant) <= 0.00001f)
	{
		return identity();
	}

	float tmp = m[0];   // copy the first element
	float invDeterminant = 1.0f / determinant;
	m[0] = invDeterminant * m[3];
	m[1] = -invDeterminant * m[1];
	m[2] = -invDeterminant * m[2];
	m[3] = invDeterminant * tmp;

	return *this;
}



///////////////////////////////////////////////////////////////////////////////
// return determinant of 3x3 matrix
///////////////////////////////////////////////////////////////////////////////
float Matrix3::getDeterminant()
{
	return m[0] * (m[4] * m[8] - m[5] * m[7]) -
		m[1] * (m[3] * m[8] - m[5] * m[6]) +
		m[2] * (m[3] * m[7] - m[4] * m[6]);
}



///////////////////////////////////////////////////////////////////////////////
// inverse 3x3 matrix
// If cannot find inverse, set identity matrix
///////////////////////////////////////////////////////////////////////////////
Matrix3& Matrix3::invert()
{
	float determinant, invDeterminant;
	float tmp[9];

	tmp[0] = m[4] * m[8] - m[5] * m[7];
	tmp[1] = m[2] * m[7] - m[1] * m[8];
	tmp[2] = m[1] * m[5] - m[2] * m[4];
	tmp[3] = m[5] * m[6] - m[3] * m[8];
	tmp[4] = m[0] * m[8] - m[2] * m[6];
	tmp[5] = m[2] * m[3] - m[0] * m[5];
	tmp[6] = m[3] * m[7] - m[4] * m[6];
	tmp[7] = m[1] * m[6] - m[0] * m[7];
	tmp[8] = m[0] * m[4] - m[1] * m[3];

	// check determinant if it is 0
	determinant = m[0] * tmp[0] + m[1] * tmp[3] + m[2] * tmp[6];
	if (fabs(determinant) <= 0.00001f)
	{
		return identity(); // cannot inverse, make it idenety matrix
	}

	// divide by the determinant
	invDeterminant = 1.0f / determinant;
	m[0] = invDeterminant * tmp[0];
	m[1] = invDeterminant * tmp[1];
	m[2] = invDeterminant * tmp[2];
	m[3] = invDeterminant * tmp[3];
	m[4] = invDeterminant * tmp[4];
	m[5] = invDeterminant * tmp[5];
	m[6] = invDeterminant * tmp[6];
	m[7] = invDeterminant * tmp[7];
	m[8] = invDeterminant * tmp[8];

	return *this;
}

///////////////////////////////////////////////////////////////////////////////
// transpose 3x3 matrix
///////////////////////////////////////////////////////////////////////////////
Matrix3& Matrix3::transpose()
{
	/*
	T[0] T[1] T[2]
	T[3] T[4] T[5]
	T[6] T[7] T[8]
	*/

	std::swap(m[1], m[3]);
	std::swap(m[2], m[6]);
	std::swap(m[5], m[7]);


	return *this;
}


///////////////////////////////////////////////////////////////////////////////
// transpose 4x4 matrix
///////////////////////////////////////////////////////////////////////////////
Matrix4& Matrix4::transpose()
{
	std::swap(m[1], m[4]);
	std::swap(m[2], m[8]);
	std::swap(m[3], m[12]);
	std::swap(m[6], m[9]);
	std::swap(m[7], m[13]);
	std::swap(m[11], m[14]);

	return *this;
}



///////////////////////////////////////////////////////////////////////////////
// inverse 4x4 matrix
///////////////////////////////////////////////////////////////////////////////
Matrix4& Matrix4::invert()
{
	// If the 4th row is [0,0,0,1] then it is affine matrix and
	// it has no projective transformation.
	if (m[12] == 0 && m[13] == 0 && m[14] == 0 && m[15] == 1)
		this->invertAffine();
	else
	{
		this->invertGeneral();
		/*@@ invertProjective() is not optimized (slower than generic one)
		if(fabs(m[0]*m[5] - m[1]*m[4]) > 0.00001f)
			this->invertProjective();   // inverse using matrix partition
		else
			this->invertGeneral();      // generalized inverse
		*/
	}

	return *this;
}



///////////////////////////////////////////////////////////////////////////////
// compute the inverse of 4x4 Euclidean transformation matrix
//
// Euclidean transformation is translation, rotation, and reflection.
// With Euclidean transform, only the position and orientation of the object
// will be changed. Euclidean transform does not change the shape of an object
// (no scaling). Length and angle are reserved.
//
// Use inverseAffine() if the matrix has scale and shear transformation.
//
// M = [ R | T ]
//     [ --+-- ]    (R denotes 3x3 rotation/reflection matrix)
//     [ 0 | 1 ]    (T denotes 1x3 translation matrix)
//
// y = M*x  ->  y = R*x + T  ->  x = R^-1*(y - T)  ->  x = R^T*y - R^T*T
// (R is orthogonal,  R^-1 = R^T)
//
//  [ R | T ]-1    [ R^T | -R^T * T ]    (R denotes 3x3 rotation matrix)
//  [ --+-- ]   =  [ ----+--------- ]    (T denotes 1x3 translation)
//  [ 0 | 1 ]      [  0  |     1    ]    (R^T denotes R-transpose)
///////////////////////////////////////////////////////////////////////////////
Matrix4& Matrix4::invertEuclidean()
{
	// transpose 3x3 rotation matrix part
	// | R^T | 0 |
	// | ----+-- |
	// |  0  | 1 |
	float tmp;
	tmp = m[1];  m[1] = m[4];  m[4] = tmp;
	tmp = m[2];  m[2] = m[8];  m[8] = tmp;
	tmp = m[6];  m[6] = m[9];  m[9] = tmp;

	// compute translation part -R^T * T
	// | 0 | -R^T x |
	// | --+------- |
	// | 0 |   0    |
	float x = m[3];
	float y = m[7];
	float z = m[11];
	m[3] = -(m[0] * x + m[1] * y + m[2] * z);
	m[7] = -(m[4] * x + m[5] * y + m[6] * z);
	m[11] = -(m[8] * x + m[9] * y + m[10] * z);

	// last row should be unchanged (0,0,0,1)

	return *this;
}



///////////////////////////////////////////////////////////////////////////////
// compute the inverse of a 4x4 affine transformation matrix
//
// Affine transformations are generalizations of Euclidean transformations.
// Affine transformation includes translation, rotation, reflection, scaling,
// and shearing. Length and angle are NOT preserved.
// M = [ R | T ]
//     [ --+-- ]    (R denotes 3x3 rotation/scale/shear matrix)
//     [ 0 | 1 ]    (T denotes 1x3 translation matrix)
//
// y = M*x  ->  y = R*x + T  ->  x = R^-1*(y - T)  ->  x = R^-1*y - R^-1*T
//
//  [ R | T ]-1   [ R^-1 | -R^-1 * T ]
//  [ --+-- ]   = [ -----+---------- ]
//  [ 0 | 1 ]     [  0   +     1     ]
///////////////////////////////////////////////////////////////////////////////
Matrix4& Matrix4::invertAffine()
{
	// R^-1
	Matrix3 r(m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]);
	r.invert();
	m[0] = r[0];  m[1] = r[1];  m[2] = r[2];
	m[4] = r[3];  m[5] = r[4];  m[6] = r[5];
	m[8] = r[6];  m[9] = r[7];  m[10] = r[8];

	// -R^-1 * T
	float x = m[3];
	float y = m[7];
	float z = m[11];
	m[3] = -(r[0] * x + r[1] * y + r[2] * z);
	m[7] = -(r[3] * x + r[4] * y + r[5] * z);
	m[11] = -(r[6] * x + r[7] * y + r[8] * z);

	// last row should be unchanged (0,0,0,1)
	//m[12] = m[13] = m[14] = 0.0f;
	//m[15] = 1.0f;

	return *this;
}



///////////////////////////////////////////////////////////////////////////////
// inverse matrix using matrix partitioning (blockwise inverse)
// It devides a 4x4 matrix into 4 of 2x2 matrices. It works in case of where
// det(A) != 0. If not, use the generic inverse method
// inverse formula.
// M = [ A | B ]    A, B, C, D are 2x2 matrix blocks
//     [ --+-- ]    det(M) = |A| * |D - ((C * A^-1) * B)|
//     [ C | D ]
//
// M^-1 = [ A' | B' ]   A' = A^-1 - (A^-1 * B) * C'
//        [ ---+--- ]   B' = (A^-1 * B) * -D'
//        [ C' | D' ]   C' = -D' * (C * A^-1)
//                      D' = (D - ((C * A^-1) * B))^-1
//
// NOTE: I wrap with () if it it used more than once.
//       The matrix is invertable even if det(A)=0, so must check det(A) before
//       calling this function, and use invertGeneric() instead.
///////////////////////////////////////////////////////////////////////////////
Matrix4& Matrix4::invertProjective()
{
	// partition
	Matrix2 a(m[0], m[1], m[4], m[5]);
	Matrix2 b(m[2], m[3], m[6], m[7]);
	Matrix2 c(m[8], m[9], m[12], m[13]);
	Matrix2 d(m[10], m[11], m[14], m[15]);

	// pre-compute repeated parts
	a.invert();             // A^-1
	Matrix2 ab = a * b;     // A^-1 * B
	Matrix2 ca = c * a;     // C * A^-1
	Matrix2 cab = ca * b;   // C * A^-1 * B
	Matrix2 dcab = d - cab; // D - C * A^-1 * B

	// check determinant if |D - C * A^-1 * B| = 0
	//NOTE: this function assumes det(A) is already checked. if |A|=0 then,
	//      cannot use this function.
	float determinant = dcab[0] * dcab[3] - dcab[1] * dcab[2];
	if (fabs(determinant) <= 0.00001f)
	{
		return identity();
	}

	// compute D' and -D'
	Matrix2 d1 = dcab;      //  (D - C * A^-1 * B)
	d1.invert();            //  (D - C * A^-1 * B)^-1
	Matrix2 d2 = -d1;       // -(D - C * A^-1 * B)^-1

	// compute C'
	Matrix2 c1 = d2 * ca;   // -D' * (C * A^-1)

	// compute B'
	Matrix2 b1 = ab * d2;   // (A^-1 * B) * -D'

	// compute A'
	Matrix2 a1 = a - (ab * c1); // A^-1 - (A^-1 * B) * C'

	// assemble inverse matrix
	m[0] = a1[0];  m[1] = a1[1];  m[2] = b1[0];  m[3] = b1[1];
	m[4] = a1[2];  m[5] = a1[3];  m[6] = b1[1];  m[2] = b1[3];
	m[8] = c1[0];  m[9] = c1[1];  m[10] = d1[0];  m[11] = d1[1];
	m[12] = c1[2];  m[13] = c1[3];  m[14] = d1[2];  m[15] = d1[3];

	return *this;
}



///////////////////////////////////////////////////////////////////////////////
// compute the inverse of a general 4x4 matrix using Cramer's Rule
// If cannot find inverse, return indentity matrix
// M^-1 = adj(M) / det(M)
///////////////////////////////////////////////////////////////////////////////
Matrix4& Matrix4::invertGeneral()
{
	// get cofactors of minor matrices
	float cofactor0 = getCofactor(m[5], m[6], m[7], m[9], m[10], m[11], m[13], m[14], m[15]);
	float cofactor1 = getCofactor(m[4], m[6], m[7], m[8], m[10], m[11], m[12], m[14], m[15]);
	float cofactor2 = getCofactor(m[4], m[5], m[7], m[8], m[9], m[11], m[12], m[13], m[15]);
	float cofactor3 = getCofactor(m[4], m[5], m[6], m[8], m[9], m[10], m[12], m[13], m[14]);

	// get determinant
	float determinant = m[0] * cofactor0 - m[1] * cofactor1 + m[2] * cofactor2 - m[3] * cofactor3;
	if (fabs(determinant) <= 0.00001f)
	{
		return identity();
	}

	// get rest of cofactors for adj(M)
	float cofactor4 = getCofactor(m[1], m[2], m[3], m[9], m[10], m[11], m[13], m[14], m[15]);
	float cofactor5 = getCofactor(m[0], m[2], m[3], m[8], m[10], m[11], m[12], m[14], m[15]);
	float cofactor6 = getCofactor(m[0], m[1], m[3], m[8], m[9], m[11], m[12], m[13], m[15]);
	float cofactor7 = getCofactor(m[0], m[1], m[2], m[8], m[9], m[10], m[12], m[13], m[14]);

	float cofactor8 = getCofactor(m[1], m[2], m[3], m[5], m[6], m[7], m[13], m[14], m[15]);
	float cofactor9 = getCofactor(m[0], m[2], m[3], m[4], m[6], m[7], m[12], m[14], m[15]);
	float cofactor10 = getCofactor(m[0], m[1], m[3], m[4], m[5], m[7], m[12], m[13], m[15]);
	float cofactor11 = getCofactor(m[0], m[1], m[2], m[4], m[5], m[6], m[12], m[13], m[14]);

	float cofactor12 = getCofactor(m[1], m[2], m[3], m[5], m[6], m[7], m[9], m[10], m[11]);
	float cofactor13 = getCofactor(m[0], m[2], m[3], m[4], m[6], m[7], m[8], m[10], m[11]);
	float cofactor14 = getCofactor(m[0], m[1], m[3], m[4], m[5], m[7], m[8], m[9], m[11]);
	float cofactor15 = getCofactor(m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]);

	// build inverse matrix = adj(M) / det(M)
	// adjugate of M is the transpose of the cofactor matrix of M
	float invDeterminant = 1.0f / determinant;
	m[0] = invDeterminant * cofactor0;
	m[1] = -invDeterminant * cofactor4;
	m[2] = invDeterminant * cofactor8;
	m[3] = -invDeterminant * cofactor12;

	m[4] = -invDeterminant * cofactor1;
	m[5] = invDeterminant * cofactor5;
	m[6] = -invDeterminant * cofactor9;
	m[7] = invDeterminant * cofactor13;

	m[8] = invDeterminant * cofactor2;
	m[9] = -invDeterminant * cofactor6;
	m[10] = invDeterminant * cofactor10;
	m[11] = -invDeterminant * cofactor14;

	m[12] = -invDeterminant * cofactor3;
	m[13] = invDeterminant * cofactor7;
	m[14] = -invDeterminant * cofactor11;
	m[15] = invDeterminant * cofactor15;

	return *this;
}



///////////////////////////////////////////////////////////////////////////////
// return determinant of 4x4 matrix
///////////////////////////////////////////////////////////////////////////////
float Matrix4::getDeterminant()
{
	return m[0] * getCofactor(m[5], m[6], m[7], m[9], m[10], m[11], m[13], m[14], m[15]) -
		m[1] * getCofactor(m[4], m[6], m[7], m[8], m[10], m[11], m[12], m[14], m[15]) +
		m[2] * getCofactor(m[4], m[5], m[7], m[8], m[9], m[11], m[12], m[13], m[15]) -
		m[3] * getCofactor(m[4], m[5], m[6], m[8], m[9], m[10], m[12], m[13], m[14]);
}



///////////////////////////////////////////////////////////////////////////////
// compute cofactor of 3x3 minor matrix without sign
// input params are 9 elements of the minor matrix
// NOTE: The caller must know its sign.
///////////////////////////////////////////////////////////////////////////////
float Matrix4::getCofactor(float m0, float m1, float m2,
	float m3, float m4, float m5,
	float m6, float m7, float m8)
{
	return m0 * (m4 * m8 - m5 * m7) -
		m1 * (m3 * m8 - m5 * m6) +
		m2 * (m3 * m7 - m4 * m6);
}



///////////////////////////////////////////////////////////////////////////////
// translate this matrix by (x, y, z)
///////////////////////////////////////////////////////////////////////////////
Matrix4& Matrix4::translate(const vec& v)
{
	return translate(v.x, v.y, v.z);
}

Matrix4& Matrix4::translate(float x, float y, float z)
{
	m[0] += m[12] * x;   m[1] += m[13] * x;   m[2] += m[14] * x;   m[3] += m[15] * x;
	m[4] += m[12] * y;   m[5] += m[13] * y;   m[6] += m[14] * y;   m[7] += m[15] * y;
	m[8] += m[12] * z;   m[9] += m[13] * z;   m[10] += m[14] * z;   m[11] += m[15] * z;
	return *this;
}



///////////////////////////////////////////////////////////////////////////////
// uniform scale
///////////////////////////////////////////////////////////////////////////////
Matrix4& Matrix4::scale(float s)
{
	return scale(s, s, s);
}

Matrix4& Matrix4::scale(float x, float y, float z)
{
	m[0] = m[0] * x;   m[1] = m[1] * x;   m[2] = m[2] * x;   m[3] = m[3] * x;
	m[4] = m[4] * y;   m[5] = m[5] * y;   m[6] = m[6] * y;   m[7] = m[7] * y;
	m[8] = m[8] * z;   m[9] = m[9] * z;   m[10] = m[10] * z;  m[11] = m[11] * z;
	return *this;
}

Matrix4& Matrix4::setDiag(float s)
{
	m[0] = s;
	  m[5] = s;   
	  m[10] =s;  
	return *this;
}



///////////////////////////////////////////////////////////////////////////////
// build a rotation matrix with given angle(degree) and rotation axis, then
// multiply it with this object
///////////////////////////////////////////////////////////////////////////////
Matrix4& Matrix4::rotate(float angle, const vec& axis)
{
	return rotate(angle, axis.x, axis.y, axis.z);
}

Matrix4& Matrix4::rotate(float angle, float x, float y, float z)
{
	float c = cosf(angle * DEG_TO_RAD);    // cosine
	float s = sinf(angle * DEG_TO_RAD);    // sine
	float xx = x * x;
	float xy = x * y;
	float xz = x * z;
	float yy = y * y;
	float yz = y * z;
	float zz = z * z;

	// build rotation matrix
	Matrix4 m;
	m[0] = xx * (1 - c) + c;
	m[1] = xy * (1 - c) - z * s;
	m[2] = xz * (1 - c) + y * s;
	m[3] = 0;
	m[4] = xy * (1 - c) + z * s;
	m[5] = yy * (1 - c) + c;
	m[6] = yz * (1 - c) - x * s;
	m[7] = 0;
	m[8] = xz * (1 - c) - y * s;
	m[9] = yz * (1 - c) + x * s;
	m[10] = zz * (1 - c) + c;
	m[11] = 0;
	m[12] = 0;
	m[13] = 0;
	m[14] = 0;
	m[15] = 1;

	// multiply it
	*this = m * (*this);

	return *this;
}

Matrix4& Matrix4::rotateX(float angle)
{
	float c = cosf(angle * DEG_TO_RAD);
	float s = sinf(angle * DEG_TO_RAD);
	float m4 = m[4], m5 = m[5], m6 = m[6], m7 = m[7],
		m8 = m[8], m9 = m[9], m10 = m[10], m11 = m[11];

	m[4] = m4 * c + m8 *-s;
	m[5] = m5 * c + m9 *-s;
	m[6] = m6 * c + m10*-s;
	m[7] = m7 * c + m11*-s;
	m[8] = m4 * s + m8 * c;
	m[9] = m5 * s + m9 * c;
	m[10] = m6 * s + m10* c;
	m[11] = m7 * s + m11* c;

	return *this;
}

Matrix4& Matrix4::rotateY(float angle)
{
	float c = cosf(angle * DEG_TO_RAD);
	float s = sinf(angle * DEG_TO_RAD);
	float m0 = m[0], m1 = m[1], m2 = m[2], m3 = m[3],
		m8 = m[8], m9 = m[9], m10 = m[10], m11 = m[11];

	m[0] = m0 * c + m8 * s;
	m[1] = m1 * c + m9 * s;
	m[2] = m2 * c + m10* s;
	m[3] = m3 * c + m11* s;
	m[8] = m0 *-s + m8 * c;
	m[9] = m1 *-s + m9 * c;
	m[10] = m2 *-s + m10* c;
	m[11] = m3 *-s + m11* c;

	return *this;
}

Matrix4& Matrix4::rotateZ(float angle)
{
	float c = cosf(angle * DEG_TO_RAD);
	float s = sinf(angle * DEG_TO_RAD);
	float m0 = m[0], m1 = m[1], m2 = m[2], m3 = m[3],
		m4 = m[4], m5 = m[5], m6 = m[6], m7 = m[7];

	m[0] = m0 * c + m4 *-s;
	m[1] = m1 * c + m5 *-s;
	m[2] = m2 * c + m6 *-s;
	m[3] = m3 * c + m7 *-s;
	m[4] = m0 * s + m4 * c;
	m[5] = m1 * s + m5 * c;
	m[6] = m2 * s + m6 * c;
	m[7] = m3 * s + m7 * c;

	return *this;
}

#define _MATRICES_
#endif // !_MATRICES_
