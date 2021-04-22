package pathfinding

import (
	"fmt"
	"testing"
)

type Path []Vert

func (p *Path) Append(x, y, z float32) {
	*p = append(*p, Vert{
		X: x,
		Y: y,
		Z: z,
	})
}

func initMesh() *NavMesh {
	var nav = new(NavMesh)
	nav.mVert = make([]Vert, 36)
	for r := 0; r < 6; r++ {
		for c := 0; c < 6; c++ {
			nav.mVert[r*6+c] = Vert{
				X: float32(c),
				Z: float32(r),
			}
		}
	}
	nav.mPoly = make([]Poly, 25)
	for r := 0; r < 5; r++ {
		for c := 0; c < 5; c++ {
			var p = Poly{
				link:    0,
				vertCnt: 4,
				areaId:  0,
				vs:      [6]int32{},
			}
			p.vs[0] = int32((r+1)*6 + c)
			p.vs[1] = int32((r+1)*6 + c + 1)
			p.vs[2] = int32(r*6 + c + 1)
			p.vs[3] = int32(r*6 + c)

			var next int32 = -1
			if c > 0 {
				nav.mLink = append(nav.mLink, Link{
					toRef: int32(r*5 + c - 1),
					next:  next,
					edge:  3,
				})
				next = int32(len(nav.mLink) - 1)
			}
			if c < 4 {
				nav.mLink = append(nav.mLink, Link{
					toRef: int32(r*5 + c + 1),
					next:  next,
					edge:  1,
				})
				next = int32(len(nav.mLink) - 1)
			}
			if r > 0 {
				nav.mLink = append(nav.mLink, Link{
					toRef: int32((r-1)*5 + c),
					next:  next,
					edge:  2,
				})
				next = int32(len(nav.mLink) - 1)
			}
			if r < 4 {
				nav.mLink = append(nav.mLink, Link{
					toRef: int32((r+1)*5 + c),
					next:  next,
					edge:  0,
				})
				next = int32(len(nav.mLink) - 1)
			}

			p.link = next
			nav.mPoly[r*5+c] = p
		}
	}
	nav.mPoly[4].areaId = 100
	nav.mPoly[8].areaId = 100
	nav.mPoly[12].areaId = 100
	nav.mPoly[16].areaId = 100
	nav.mPoly[23].areaId = 100
	return nav
}

func TestHeight(t *testing.T) {
	var (
		v0 = Vert{X: 0, Z: 0}
		v1 = Vert{X: 1, Z: 0}
		v2 = Vert{X: 0, Z: 1}
		p  = Vert{X: 0.2, Y: 1, Z: 0.2}
	)
	fmt.Println(vHeightOnTriangle(p, v0, v1, v2))
}

func TestNavMesh_LocatePoly(t *testing.T) {
	var nav = initMesh()
	fmt.Println(nav.LocatePoly(Vert{
		X: 0.5,
		Y: 0.5,
	}))
}

func TestNavMesh(t *testing.T) {
	var nav = initMesh()
	var q = NewQuery(nav, 100)
	var p Path
	fmt.Println(q.FindPath(Vert{
		X: 0.5,
		Z: 0.5,
	}, Vert{
		X: 4.5,
		Z: 4.5,
	}, &p))

	fmt.Println(p)
}