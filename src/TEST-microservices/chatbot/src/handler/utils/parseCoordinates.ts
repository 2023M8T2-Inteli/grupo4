import { Point, Tool } from "@prisma/client"

interface ParsedCoordinates {
    parsedTools: string,
    parsedLocations: string

}

const parseCoordinates = (tools: Tool[], locations: Point[]): ParsedCoordinates => {

    const parsedTools = tools.map((coordinate) => {
        const { name, pointX, pointY } = coordinate
        return `${name}: [${pointX}, ${pointY}]`
    }).join("\n")

    const parsedLocations = locations.map((coordinate) => {
        const { name, pointX, pointY } = coordinate
        return `${name}: [${pointX}, ${pointY}]`
    }).join('\n')

    return {parsedTools, parsedLocations}

}

export default parseCoordinates
