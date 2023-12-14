import { Point, Tool } from "@prisma/client"

interface ParsedCoordinates {
    parsedToolCoordinates: string,
    parsedLocationCoordinates: string

}

const parseCoordinates = (tools: Tool[], locations: Point[]): ParsedCoordinates => {

    const parsedTools = tools.map((coordinate) => {
        const { name, x, y } = coordinate
        return `${name} - [${x}, ${y}]`
    })

    const parsedLocations = locations.map((coordinate) => {
        const { name, x, y } = coordinate
        return `${name} - [${x}, ${y}]`
    })

    return {parsedTools.join('\n'), parsedLocations.join('\n')}

}

export default parseCoordinates