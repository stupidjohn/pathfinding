package pathfinding

import "fmt"

type Query struct {
	nodePath  []int32
	mesh      *NavMesh
	nodePool  *dtNodePool
	nodeQueue *dtNodeQueue
}

type QueryResult interface {
	Append(x, y, z float32)
}

func NewQuery(mesh *NavMesh, size int32) *Query {
	var q = new(Query)
	q.mesh = mesh
	q.nodePool = newNodePool(size)
	q.nodeQueue = newNodeQueue(size)
	return q
}

func (q *Query) clear() {
	q.nodePool.clear()
	q.nodeQueue.clear()
}

func (q *Query) FindPath(startPos, endPos Vert, result QueryResult) bool {
	var (
		startRef, endRef int32
		ok               bool
	)
	startRef, startPos, ok = q.mesh.LocatePoly(startPos)
	endRef, endPos, _ = q.mesh.LocatePoly(endPos)
	fmt.Println(startPos, endPos)
	q.clear()
	if _, _, ok = q.findPath(startRef, endRef, startPos, endPos); !ok {
		return false
	}
	return q.pullPath(startRef, endRef, startPos, endPos, result)
}

func (q *Query) pullPath(startRef, endRef int32, startPos, endPos Vert, result QueryResult) bool {
	result.Append(startPos.X, startPos.Y, startPos.Z)
	if len(q.nodePath) > 1 {
		var (
			portalApex, portalLeft, portalRight Vert
			left, right                         Vert
			ok                                  bool
		)
		for i := 0; i < len(q.nodePath); i++ {
			if i+1 < len(q.nodePath) {
				if left, right, ok = q.mesh.getPortal(q.nodePath[i], q.nodePath[i+1]); !ok {
					return false
				}
				// portalApex on segment of portal
				if _, d := distPtSegSqr2D(portalApex, left, right); d < eqs {
					continue
				}
			} else {
				left, right = endPos, endPos
			}
			// handle left
			if triArea2D(portalApex, portalLeft, left) >= 0 { // tighten left
				if vEqual(portalApex, portalLeft) || triArea2D(portalApex, portalRight, left) < eps*eps {
					portalLeft = left
				} else {
					portalApex = portalRight
					result.Append(portalApex.X, portalApex.Y, portalApex.Z)
					portalLeft, portalRight = portalApex, portalApex
				}
			}
			// handle right
			if triArea2D(portalApex, portalRight, right) <= 0 { //tighten right
				if vEqual(portalApex, portalRight) || triArea2D(portalApex, portalLeft, right) > -eps*eps {
					portalRight = right
				} else {
					portalApex = portalLeft
					result.Append(portalApex.X, portalApex.Y, portalApex.Z)
					portalLeft, portalRight = portalApex, portalApex
				}
			}
		}
	}
	result.Append(endPos.X, endPos.Y, endPos.Z)
	return true
}

func (q *Query) findPath(startRef, endRef int32, startPos, endPos Vert) (
	partial, outOfNodes, success bool) {
	var (
		startNode        = q.nodePool.getNode(startRef)
		lastBestNodeCost = vDist(startPos, endPos)
		lastBestNode     = startNode
		bestNode         *dtNode
		neighborNode     *dtNode
		bestPoly         *Poly
		parentRef        int32

		cost, heuristic, total float32
	)
	*startNode = dtNode{
		pos:    startPos,
		pIdx:   -1,
		ref:    startRef,
		status: nodeOpen,
		total:  lastBestNodeCost,
	}
	q.nodeQueue.push(startNode)
	for !q.nodeQueue.empty() {
		bestNode = q.nodeQueue.pop()
		if bestNode.ref == endRef {
			lastBestNode = bestNode
			break
		}
		bestPoly = &q.mesh.mPoly[bestNode.ref]
		parentRef = -1
		if bestNode.pIdx >= 0 {
			parentRef = q.nodePool.getNodeAtIdx(bestNode.pIdx).ref
		}
		for l := bestPoly.link; l != -1; l = q.mesh.mLink[l].next {
			var neighborRef = q.mesh.mLink[l].toRef
			if neighborRef == parentRef { // skip parent
				continue
			}
			if q.mesh.mPoly[neighborRef].areaId != bestPoly.areaId {
				continue
			}
			neighborNode = q.nodePool.getNode(neighborRef)
			if neighborNode == nil {
				outOfNodes = true
				continue
			}
			// first visit, calculate pos
			if neighborNode.status == 0 {
				neighborNode.pos, _ = q.mesh.edgeMidPoint(bestNode.ref, neighborRef)
			}

			// calculate cost + heuristic
			if neighborRef == endRef {
				cost = bestNode.cost + vDist(bestNode.pos, neighborNode.pos) + vDist(neighborNode.pos, endPos)
				heuristic = 0
			} else {
				cost = bestNode.cost + vDist(bestNode.pos, neighborNode.pos)
				heuristic = vDist(neighborNode.pos, endPos)
			}
			total = cost + heuristic

			// skip non-optimal node
			switch neighborNode.status {
			case nodeOpen, nodeClose:
				if neighborNode.total <= total {
					continue
				}
			}

			neighborNode.pIdx = q.nodePool.getIdx(bestNode)
			neighborNode.status &= ^nodeClose
			neighborNode.cost = cost
			neighborNode.total = total

			switch neighborNode.status {
			case 0:
				neighborNode.status = nodeOpen
				q.nodeQueue.push(neighborNode)
			case nodeOpen:
				q.nodeQueue.fix(neighborNode)
			}
			if heuristic < lastBestNodeCost {
				lastBestNodeCost = heuristic
				lastBestNode = neighborNode
			}
		}
	}
	if lastBestNode.ref != endRef {
		partial = true
	}
	success = q.retrievePath(lastBestNode)
	return
}

func (q *Query) retrievePath(endNode *dtNode) bool {
	var (
		curNode = endNode
		length  = 0
	)
	for curNode != nil {
		length++
		curNode = q.nodePool.getNodeAtIdx(curNode.pIdx)
	}
	if cap(q.nodePath) < length {
		q.nodePath = make([]int32, length)
	} else {
		q.nodePath = q.nodePath[0:length]
	}

	curNode = endNode
	for i := length - 1; i >= 0; i-- {
		q.nodePath[i] = curNode.ref
		curNode = q.nodePool.getNodeAtIdx(curNode.pIdx)
	}
	return true
}