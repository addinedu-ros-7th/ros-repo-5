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

    total_price = 0.0
    for product_id, qty in req.cart.items():
        sql = "SELECT price FROM products WHERE product_id=%s"
        rows = db.execute_query(sql, (product_id,))
        if len(rows) > 0:
            price = rows[0][0]
            total_price += (price * qty)

    # 주문 정보 DB insert
    insert_order_sql = """
    INSERT INTO orders (order_id, user_id, order_status, price)
    VALUES (%s, %s, %s, %s)
    """
    db.execute_query(insert_order_sql, (order_id, req.userId, "queued", total_price))

    for product_id, qty in req.cart.items():
        insert_item_sql = """
        INSERT INTO order_items (order_id, product_id, quantity)
        VALUES (%s, %s, %s)
        """
        db.execute_query(insert_item_sql, (order_id, product_id, qty))

    # Order 도메인 객체 생성 및 로봇 스케줄링
    o = Order(req.userId, order_id, req.cart)
    assigned_robot_id = order_service.assign_order(o)

    if assigned_robot_id is None:
        # 로봇이 없어서 queued
        return {
            "status": "queued",
            "orderId": order_id,
            "message": "No available robot. Order queued."
        }
    else:
        # 바로 할당됨
        update_sql = "UPDATE orders SET order_status='assigned' WHERE order_id=%s"
        db.execute_query(update_sql, (order_id,))

        return {
            "status": "assigned",
            "orderId": order_id,
            "message": "Order created successfully"
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
    db = request.app.state.db
    if db is None:
        raise HTTPException(status_code=500, detail="DB Manager not set")

    sql = "SELECT robot_id, type, name, waiting_time FROM robots"
    rows = db.execute_query(sql)

    results = []
    for (robot_id, rtype, name, waiting_time) in rows:
        if rtype == "주행":
            status = "주문처리중"
            remaining_items = {"item1": 3, "item2": 2}
            carrying_items = {"item3": 3, "item4": 2}
            results.append({
                "id": robot_id,
                "type": rtype,
                "status": status,
                "remainingItems": remaining_items,
                "carryingItems": carrying_items
            })
        else:
            status = "대기중"
            results.append({
                "id": robot_id,
                "type": rtype,
                "status": status
            })
    return {"robots": results}


@router.get("/api/robots/locations")
def get_robot_locations(request: Request) -> dict:
    db = request.app.state.db
    if db is None:
        raise HTTPException(status_code=500, detail="DB Manager not set")

    sql_robots = "SELECT robot_id FROM robots"
    rows_robots = db.execute_query(sql_robots)

    locations = []
    for (robot_id,) in rows_robots:
        sql_log = """
        SELECT location
        FROM deli_bot_logs
        WHERE robot_id=%s
        ORDER BY time DESC
        LIMIT 1
        """
        logs = db.execute_query(sql_log, (robot_id,))
        if len(logs) == 0 or not logs[0][0]:
            x, y = (0.0, 0.0)
        else:
            loc_str = logs[0][0]
            parts = loc_str.split(",")
            if len(parts) == 2:
                x, y = float(parts[0]), float(parts[1])
            else:
                x, y = (0.0, 0.0)

        locations.append({
            "id": robot_id,
            "x": x,
            "y": y
        })

    return {"locations": locations}


@router.get("/api/robots/logs")
def get_robots_logs(request: Request) -> dict:
    db = request.app.state.db
    if db is None:
        raise HTTPException(status_code=500, detail="DB Manager not set")

    sql = """
    SELECT robot_id, status as event, time
    FROM deli_bot_logs
    UNION ALL
    SELECT robot_id, status as event, time
    FROM deli_arm_logs
    ORDER BY time
    """
    rows = db.execute_query(sql)

    results = []
    for (rid, evt, t) in rows:
        iso_str = t.strftime("%Y-%m-%dT%H:%M:%SZ")
        results.append({
            "robotId": rid,
            "event": evt,
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
