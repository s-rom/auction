function parsePoint(pointStr){
    let len = pointStr.length;
    let delimIdx = pointStr.indexOf(",");
    let xStr = pointStr.substring(1, delimIdx);
    let yStr = pointStr.substring(delimIdx+1, len-1);

    let x = parseFloat(xStr);
    let y = parseFloat(yStr);

    return [x, y];
}