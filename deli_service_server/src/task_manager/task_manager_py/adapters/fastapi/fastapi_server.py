# task_manager_py/adapters/fastapi/fastapi_server.py

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel
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
# 3) 로봇 (Robot) 관련 모델
# -------------------------
class RobotWaitingTimeRequest(BaseModel):
    robotId: str
    waitingTime: int

# -------------------------
# 4) 로그인 관련 모델
# -------------------------
class LoginRequest(BaseModel):
    userId: str
    password: str


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

    if userId:
        sql_orders = "SELECT order_id, user_id, order_status, price FROM orders WHERE user_id=%s"
        rows_orders = db.execute_query(sql_orders, (userId,))
    else:
        sql_orders = "SELECT order_id, user_id, order_status, price FROM orders"
        rows_orders = db.execute_query(sql_orders)

    results = []
    for (oid, uid, status, price) in rows_orders:
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
            "price": float(price if price else 0.0)
        })
    return {"orders": results}


@router.delete("/api/orders/{orderId}")
def cancel_order(orderId: str, request: Request) -> dict:
    db = request.app.state.db
    if db is None:
        raise HTTPException(status_code=500, detail="DB Manager not set")

    check_sql = "SELECT order_id FROM orders WHERE order_id=%s"
    rows = db.execute_query(check_sql, (orderId,))
    if len(rows) == 0:
        raise HTTPException(status_code=404, detail="Order not found")

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
        # 로봇 바쁜 상태인지
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
    """
    도메인 모델에 Robot.location = (x,y)가 있다고 가정.
    각 로봇의 현재 위치 반환.
    """
    print(">>> /api/robots/locations called!")
    order_service = request.app.state.order_service
    if not order_service:
        print("!!! order_service is None.")
        raise HTTPException(status_code=500, detail="OrderService not set")

    print(f">>> order_service.robots: {order_service.robots}")
    locations = []
    for r_id, robot_obj in order_service.robots.items():
        x, y = robot_obj.location
        locations.append({
            "robotId": r_id,
            "x": x,
            "y": y
        })

    print(f">>> returning locations: {locations}")
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

    # UNION ALL 쿼리
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
        """
        deli_bot_logs  -> (robot_id, status, location, time)
        deli_arm_logs  -> (robot_id, status, NULL, time)
        """
        robot_id, status, location, timestamp = row
        # 시간을 ISO 포맷으로 (예: 2023-02-09T18:44:00Z)
        iso_str = timestamp.strftime("%Y-%m-%dT%H:%M:%SZ")

        results.append({
            "robotId": robot_id,
            "status": status,
            "location": location if location else "",
            "timestamp": iso_str
        })

    return {"logs": results}


@router.post("/api/robots/waiting-time")
def update_robot_waiting_time(req: RobotWaitingTimeRequest, request: Request) -> dict:
    db = request.app.state.db
    if db is None:
        raise HTTPException(status_code=500, detail="DB Manager not set")

    check_sql = "SELECT robot_id FROM robots WHERE robot_id=%s"
    rows = db.execute_query(check_sql, (req.robotId,))
    if len(rows) == 0:
        raise HTTPException(status_code=404, detail="Robot not found")

    update_sql = "UPDATE robots SET waiting_time=%s WHERE robot_id=%s"
    db.execute_query(update_sql, (req.waitingTime, req.robotId))

    return {"status": "success", "message": "Waiting time updated successfully"}
