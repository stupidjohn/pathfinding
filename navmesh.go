package pathfinding

const maxVertPerPoly = 6

type Poly struct {
	link    int32 // first link
	vertCnt int32
	areaId  int32
	vs      [maxVertPerPoly]int32
}

type Link struct {
	toRef int32 // connect which poly [must exist]
	next  int32 // next link [-1 if no next]
	edge  int32 // which edge this link belongs, 0->(0,1) 1->(1,2)... n-1->(n-1,0)
}

type NavMesh struct {
	mVert []Vert
	mPoly []Poly
	mLink []Link
}

func (n *NavMesh) getPortal(fromRef, toRef int32) (left, right Vert, ok bool) {
	for l := n.mPoly[fromRef].link; l != -1; l = n.mLink[l].next {
		if n.mLink[l].toRef == toRef {
			var poly = &n.mPoly[fromRef]
			var link = &n.mLink[l]
			var v0 = n.mVert[poly.vs[link.edge]]
			var v1 = n.mVert[poly.vs[(link.edge+1)%poly.vertCnt]]
			return v0, v1, true
		}
	}
	return NilVert, NilVert, false
}

func (n *NavMesh) edgeMidPoint(fromRef, toRef int32) (p Vert, ok bool) {
	var (
		v0, v1 Vert
	)
	v0, v1, ok = n.getPortal(fromRef, toRef)
	if !ok {
		return
	}
	return Vert{
		X: (v0.X + v1.X) * 0.5,
		Y: (v0.Y + v1.Y) * 0.5,
		Z: (v0.Z + v1.Z) * 0.5,
	}, true
}

func (n *NavMesh) GetPolyHeight(poly *Poly, p Vert) (height float32, ok bool) {
	var v0 = n.mVert[poly.vs[0]]
	for i := int32(1); i < poly.vertCnt-1; i++ {
		var (
			v1 = n.mVert[poly.vs[i]]
			v2 = n.mVert[poly.vs[i+1]]
		)
		if height, ok = vHeightOnTriangle(p, v0, v1, v2); ok {
			return
		}
	}
	return 0, false
}

func (n *NavMesh) closestPointOnPoly(poly *Poly, p Vert) Vert {
	var (
		l, r   Vert
		bd, bt float32
	)
	for i := int32(0); i < poly.vertCnt; i++ {
		var (
			v0   = n.mVert[poly.vs[i]]
			v1   = n.mVert[poly.vs[(i+1)%poly.vertCnt]]
			t, d = distPtSegSqr2D(p, v0, v1)
		)
		if d < bd {
			bd, bt = d, t
			l, r = v0, v1
		}
	}
	return vInter(l, r, bt)
}

func (n *NavMesh) LocatePoly(p Vert) (polyRef int32, pt Vert, onPoly bool) {
	var (
		bd float32
	)
	for i := 0; i < len(n.mPoly); i++ {
		if h, ok := n.GetPolyHeight(&n.mPoly[i], p); ok {
			return int32(i), Vert{
				X: p.X,
				Y: h,
				Z: p.Z,
			}, true
		}
		c := n.closestPointOnPoly(&n.mPoly[i], p)
		d := vSqr(vSub(p, c))
		if d < bd {
			polyRef = int32(i)
			pt = c
		}
	}
	return
}