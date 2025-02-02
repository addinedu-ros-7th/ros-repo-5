from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel, Field
from typing import Dict, Optional, List

from task_manager_py.domain.models.order import Order

router = APIRouter()

# -------------------------
# 1) 유저 (User) 관련 모델
# -------------------------
class CreateUserRequest(BaseModel):
    name: str
    id: str
    password: str
    address: str
    email: str

class UpdateUserRequest(BaseModel):
    name: Optional[str] = None
    address: Optional[str] = None
    email: Optional[str] = None

# -------------------------
# 2) 주문 (Order) 관련 모델
# -------------------------
class CreateOrderRequest(BaseModel):
    userId: str
    cart: Dict[str, int]

# -------------------------
# 3) 매대 점유 시간 관련 모델
# -------------------------
class ShelfWaitingTimeRequest(BaseModel):
    normal: int = Field(alias="일반 매대")
    cold: int = Field(alias="냉동 매대")
    fresh: int = Field(alias="신선 매대")

# -------------------------
# 4) 로그인 관련 모델
# -------------------------
class LoginRequest(BaseModel):
    userId: str
    password: str


# -------------------------
# 5) 사람 매대 점유 여부 모델
# -------------------------
class PersonOnShelfRequest(BaseModel):
    일반매대: bool = Field(alias="일반 매대")
    냉동매대: bool = Field(alias="냉동 매대")
    신선매대: bool = Field(alias="신선 매대")

# ------------------------------------------------------------------------------
# [Login API]
# ------------------------------------------------------------------------------
@router.post("/api/login")
def login(req: LoginRequest, request: Request) -> dict:
    db = request.app.state.db
    if db is None:
        raise HTTPException(status_code=500, detail="DB Manager not set")

    sql = "SELECT password_hash FROM users WHERE user_id=%s"
    rows = db.execute_query(sql, (req.userId,))
    if len(rows) == 0:
        raise HTTPException(status_code=404, detail="User not found")

    stored_password = rows[0][0]
    if stored_password != req.password:
        raise HTTPException(status_code=401, detail="Invalid password")

    return {"status": "success", "message": "Login successful"}


# ------------------------------------------------------------------------------
# [User APIs]
# ------------------------------------------------------------------------------
@router.post("/api/users")
def create_user(req: CreateUserRequest, request: Request) -> dict:
    db = request.app.state.db
    if db is None:
        raise HTTPException(status_code=500, detail="DB Manager not set")

    try:
        insert_sql = """
            INSERT INTO users (user_id, name, email, address, password_hash)
            VALUES (%s, %s, %s, %s, %s)
        """
        db.execute_query(insert_sql, (req.id, req.name, req.email, req.address, req.password))
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"User creation failed: {str(e)}")

    return {"status": "success", "message": "User created successfully"}

@router.delete("/api/users/{userId}")
def delete_user(userId: str, request: Request) -> dict:
    db = request.app.state.db
    if db is None:
        raise HTTPException(status_code=500, detail="DB Manager not set")

    check_sql = "SELECT user_id FROM users WHERE user_id=%s"
    rows = db.execute_query(check_sql, (userId,))
    if len(rows) == 0:
        raise HTTPException(status_code=404, detail="User not found")

    delete_sql = "DELETE FROM users WHERE user_id=%s"
    db.execute_query(delete_sql, (userId,))

    return {"status": "success", "message": "User deleted successfully"}

@router.get("/api/users/{userId}")
def get_user_info(userId: str, request: Request) -> dict:
    db = request.app.state.db
    if db is None:
        raise HTTPException(status_code=500, detail="DB Manager not set")

    sql = "SELECT user_id, name, email, address FROM users WHERE user_id=%s"
    rows = db.execute_query(sql, (userId,))
    if len(rows) == 0:
        raise HTTPException(status_code=404, detail="User not found")

    user_id, name, email, address = rows[0]
    return {
        "name": name,
        "id": user_id,
        "address": address,
        "email": email
    }

@router.post("/api/users/{userId}")
def update_user_info(userId: str, req: UpdateUserRequest, request: Request) -> dict:
    db = request.app.state.db
    if db is None:
        raise HTTPException(status_code=500, detail="DB Manager not set")

    check_sql = "SELECT user_id FROM users WHERE user_id=%s"
    rows = db.execute_query(check_sql, (userId,))
    if len(rows) == 0:
        raise HTTPException(status_code=404, detail="User not found")

    fields = []
    values = []

    if req.name is not None:
        fields.append("name=%s")
        values.append(req.name)
    if req.address is not None:
        fields.append("address=%s")
        values.append(req.address)
    if req.email is not None:
        fields.append("email=%s")
        values.append(req.email)

    if not fields:
        return {"status": "success", "message": "Nothing to update"}

    update_sql = f"UPDATE users SET {', '.join(fields)} WHERE user_id=%s"
    values.append(userId)
    db.execute_query(update_sql, tuple(values))

    return {"status": "success", "message": "User updated successfully"}

@router.get("/api/users")
def get_all_users(request: Request) -> dict:
    db = request.app.state.db
    if db is None:
        raise HTTPException(status_code=500, detail="DB Manager not set")

    sql = "SELECT user_id, name, email, address FROM users"
    rows = db.execute_query(sql)

    results = []
    for (user_id, name, email, address) in rows:
        results.append({
            "name": name,
            "id": user_id,
            "address": address,
            "email": email
        })

    return {"users": results}


# ------------------------------------------------------------------------------
# [Order APIs]
# ------------------------------------------------------------------------------
@router.post("/api/orders")
def create_order(req: CreateOrderRequest, request: Request) -> dict:
    """
    주문 생성
    POST /api/orders
    body:
      { "userId": "user01", "cart": { "냉동1": 3, "신선2": 2 } }
    """
    db = request.app.state.db
    order_service = request.app.state.order_service
    if db is None or order_service is None:
        raise HTTPException(status_code=500, detail="Service or DB not set")

    import uuid
    order_id = "order_" + str(uuid.uuid4())[:8]

    # 총 가격 계산
    total_price = 0.0
    for product_id, qty in req.cart.items():
        sql = "SELECT price FROM products WHERE product_id=%s"
        rows = db.execute_query(sql, (product_id,))
        if len(rows) > 0:
            price = rows[0][0]
            total_price += (price * qty)

    # 주문 정보: 기본 상태를 'queued'로 INSERT
    insert_order_sql = """
    INSERT INTO orders (order_id, user_id, order_status, price)
    VALUES (%s, %s, %s, %s)
    """
    db.execute_query(insert_order_sql, (order_id, req.userId, "queued", total_price))

    # 주문 아이템 정보 INSERT
    for product_id, qty in req.cart.items():
        insert_item_sql = """
        INSERT INTO order_items (order_id, product_id, quantity)
        VALUES (%s, %s, %s)
        """
        db.execute_query(insert_item_sql, (order_id, product_id, qty))

    # Order 객체 생성 및 로봇 할당 시도
    o = Order(req.userId, order_id, req.cart)
    assigned_robot_id = order_service.assign_order(o)

    if assigned_robot_id is None:
        # 로봇 할당 불가 -> 상태 유지(queued 상태)
        return {
            "status": "queued",
            "orderId": order_id,
            "message": "No available robot. Order queued."
        }
    else:
        # 로봇 할당 성공 -> 주문 상태를 'assigned'로 업데이트
        update_sql = "UPDATE orders SET order_status = %s WHERE order_id = %s"
        db.execute_query(update_sql, ("assigned", order_id))

        return {
            "status": "assigned",
            "orderId": order_id,
            "message": "Order created and assigned successfully"
        }



@router.get("/api/orders")
def get_orders(userId: Optional[str] = None, request: Request = None) -> dict:
    db = request.app.state.db
    if db is None:
        raise HTTPException(status_code=500, detail="DB Manager not set")

    # created_at, over_at 컬럼을 추가해서 조회
    if userId:
        sql_orders = """
            SELECT order_id, user_id, order_status, price, created_at, over_at
            FROM orders
            WHERE user_id=%s
        """
        rows_orders = db.execute_query(sql_orders, (userId,))
    else:
        sql_orders = """
            SELECT order_id, user_id, order_status, price, created_at, over_at
            FROM orders
        """
        rows_orders = db.execute_query(sql_orders)

    results = []
    # 쿼리 결과에서 created_at, over_at을 함께 받도록 언패킹
    for (oid, uid, status, price, created_at, over_at) in rows_orders:
        sql_items = "SELECT product_id, quantity FROM order_items WHERE order_id=%s"
        rows_items = db.execute_query(sql_items, (oid,))

        items_dict = {}
        for (pid, qty) in rows_items:
            items_dict[pid] = qty

        results.append({
            "orderId": oid,
            "status": status,
            "userId": uid,
            "items": items_dict,
            "price": float(price if price else 0.0),
            "createdAt": created_at,  # created_at 필드 추가
            "overAt": over_at         # over_at 필드 추가
        })
    return {"orders": results}

@router.delete("/api/orders/{orderId}")
def cancel_order(orderId: str, request: Request) -> dict:
    db = request.app.state.db
    if db is None:
        raise HTTPException(status_code=500, detail="DB Manager not set")

    # (1) DB에서 해당 주문 존재 여부 확인
    check_sql = "SELECT order_id FROM orders WHERE order_id=%s"
    rows = db.execute_query(check_sql, (orderId,))
    if len(rows) == 0:
        raise HTTPException(status_code=404, detail="Order not found")

    # (2) OrderService에서 큐에 있는 주문 제거
    #     order_queue 에 담긴 Order 객체 중, order_id가 일치하는 것을 제거
    order_service = request.app.state.order_service
    if order_service:
        for i, queued_order in enumerate(order_service.order_queue):
            if queued_order.order_id == orderId:
                del order_service.order_queue[i]
                break

    # (3) DB에서 order_items, orders 테이블에서 삭제
    del_items_sql = "DELETE FROM order_items WHERE order_id=%s"
    db.execute_query(del_items_sql, (orderId,))

    del_order_sql = "DELETE FROM orders WHERE order_id=%s"
    db.execute_query(del_order_sql, (orderId,))

    return {"status": "success", "message": "Order canceled successfully"}


# ------------------------------------------------------------------------------
# [Robots APIs]
# ------------------------------------------------------------------------------
@router.get("/api/robots/status")
def get_robots_status(request: Request) -> dict:
    """
    도메인 모델의 Robot 객체 정보(상태)를 조회.
    - battery_level
    - waiting_time
    - busy 여부
    - remaining_items
    - carrying_items
    - ...
    """
    order_service = request.app.state.order_service
    if not order_service:
        raise HTTPException(status_code=500, detail="OrderService not set")

    results = []
    for r_id, robot_obj in order_service.robots.items():
        status_str = "주문처리중" if robot_obj.busy else "대기중"
        data = {
            "robotId": robot_obj.robot_id,
            "name": robot_obj.name,
            "type": robot_obj.robot_type,
            "batteryLevel": robot_obj.battery_level,
            "waitingTime": robot_obj.waiting_time,
            "status": status_str,
            "location": {
                "x": robot_obj.location[0],
                "y": robot_obj.location[1]
            },
            "remainingItems": robot_obj.remaining_items,
            "carryingItems": robot_obj.carrying_items
        }
        results.append(data)

    return {"robots": results}


@router.get("/api/robots/locations")
def get_robot_locations(request: Request) -> dict:
    order_service = request.app.state.order_service
    if not order_service:
        raise HTTPException(status_code=500, detail="OrderService not set")

    locations = []
    for r_id, robot_obj in order_service.robots.items():
        x, y = robot_obj.location
        locations.append({
            "robotId": r_id,
            "x": x,
            "y": y
        })
    return {"locations": locations}


@router.get("/api/robots/logs")
def get_robots_logs(request: Request) -> dict:
    """
    deli_bot_logs (주행 로봇 로그), deli_arm_logs (로봇팔 로그)
    두 테이블을 UNION ALL 한 뒤, 시간 순으로 정렬하여 반환.
    """
    db = request.app.state.db
    if not db:
        raise HTTPException(status_code=500, detail="DB Manager not set")

    sql = """
    SELECT robot_id, status, location, time
    FROM deli_bot_logs
    UNION ALL
    SELECT robot_id, status, NULL as location, time
    FROM deli_arm_logs
    ORDER BY time
    """
    rows = db.execute_query(sql)

    results = []
    for row in rows:
        robot_id, status, location, timestamp = row
        iso_str = timestamp.strftime("%Y-%m-%dT%H:%M:%SZ")

        results.append({
            "robotId": robot_id,
            "status": status,
            "location": location if location else "",
            "timestamp": iso_str
        })

    return {"logs": results}


@router.post("/api/robots/waiting-time")
def update_shelf_waiting_time(req: ShelfWaitingTimeRequest, request: Request) -> dict:
    """
    매대 대기 필요시간 (사람) 입력 API
    {
      "일반 매대": 13,
      "냉동 매대": 0,
      "신선 매대": 0
    }
    -> occupied_info[station]["person"] 의 remain_time을 += ...
    """
    order_service = request.app.state.order_service
    if not order_service:
        raise HTTPException(status_code=500, detail="OrderService not set")

    occupied_info = order_service.occupied_info

    # (1) 일반 매대
    person_occ, person_time = occupied_info["일반"]["person"]
    new_time = person_time + req.normal
    if new_time > 0:
        person_occ = True
    occupied_info["일반"]["person"] = (person_occ, new_time)

    # (2) 냉동 매대
    person_occ, person_time = occupied_info["냉동"]["person"]
    new_time = person_time + req.cold
    if new_time > 0:
        person_occ = True
    occupied_info["냉동"]["person"] = (person_occ, new_time)

    # (3) 신선 매대
    person_occ, person_time = occupied_info["신선"]["person"]
    new_time = person_time + req.fresh
    if new_time > 0:
        person_occ = True
    occupied_info["신선"]["person"] = (person_occ, new_time)

    return {
        "status": "success",
        "message": "Waiting time updated successfully"
    }


# ------------------------------------------------------------------------------
# [shelf occupied by person]
# ------------------------------------------------------------------------------
@router.post("/api/persononshelf")
def update_person_on_shelf(req: PersonOnShelfRequest, request: Request) -> dict:
    """
    yolo 등에서 매대 주변에 사람 존재 여부를 실시간으로 알려줄 때 사용.
    예)
    {
      "일반 매대": true,
      "냉동 매대": false,
      "신선 매대": false
    }
    """
    order_service = request.app.state.order_service
    if not order_service:
        raise HTTPException(status_code=500, detail="OrderService not set")

    occupied_info = order_service.occupied_info

    # 일반 매대
    person_occ, person_time = occupied_info["일반"]["person"]
    if person_time > 0:
        # 남은시간이 남아있으면 True 유지
        new_occ = True
    else:
        # 시간 0이면 YOLO 결과를 그대로 반영
        new_occ = req.일반매대
    occupied_info["일반"]["person"] = (new_occ, person_time)

    # 냉동 매대
    person_occ, person_time = occupied_info["냉동"]["person"]
    if person_time > 0:
        new_occ = True
    else:
        new_occ = req.냉동매대
    occupied_info["냉동"]["person"] = (new_occ, person_time)

    # 신선 매대
    person_occ, person_time = occupied_info["신선"]["person"]
    if person_time > 0:
        new_occ = True
    else:
        new_occ = req.신선매대
    occupied_info["신선"]["person"] = (new_occ, person_time)

    return {
        "status": "success",
        "message": "person on shelf updated successfully"
    }
