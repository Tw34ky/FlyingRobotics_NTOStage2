import requests as rq


HOST = ''
PORT = ''


def get_markers() -> dict:
    return rq.get(f"http://{HOST}:{PORT}/markers").json()

def get_targets():
    return rq.get(f"http://{HOST}:{PORT}/targets").json()

def get_map() -> dict:
    return rq.get(f"http://{HOST}:{PORT}/map").json()

def get_position() -> dict:
    return rq.get(f"http://{HOST}:{PORT}/coords").json()


def update_graph(graph_figure: dict) -> dict:
    """Обновление графика с позицией дрона и маркерами"""
    points = get_position()
    markers = get_markers()
    
    x = points["x"]
    y = points["y"]
    graph_figure['data'][0].update(x=[x])
    graph_figure['data'][0].update(y=[y])
    
    for marker in markers["markers"]:
        graph_figure['data'].append({
            "marker": {
                "color": marker["color"], 
                "size": marker["size"], 
                "symbol": marker["type"]
            }, 
            "mode": "markers", 
            "type": "scatter", 
            "x": [marker["x"]], 
            "y": [marker["y"]]
        })
    return graph_figure

